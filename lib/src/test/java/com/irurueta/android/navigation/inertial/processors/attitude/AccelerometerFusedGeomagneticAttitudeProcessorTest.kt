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
package com.irurueta.android.navigation.inertial.processors.attitude

import android.location.Location
import com.irurueta.android.navigation.inertial.QuaternionHelper
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.AccelerationUnit
import io.mockk.*
import org.junit.After
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import java.util.*
import kotlin.math.abs

@RunWith(RobolectricTestRunner::class)
class AccelerometerFusedGeomagneticAttitudeProcessorTest {

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun constructor_whenNoParameters_returnsExpectedValues() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        assertNull(processor.processorListener)
        assertEquals(Quaternion(), processor.fusedAttitude)
        assertEquals(0.0, processor.gx, 0.0)
        assertEquals(0.0, processor.gy, 0.0)
        assertEquals(0.0, processor.gz, 0.0)
        val gravity1 = processor.gravity
        assertEquals(AccelerationTriad(), gravity1)
        val gravity2 = AccelerationTriad()
        processor.getGravity(gravity2)
        assertEquals(gravity1, gravity2)
        assertNull(processor.location)
        assertTrue(processor.adjustGravityNorm)
        assertNull(processor.currentDate)
        assertFalse(processor.useAccurateLevelingProcessor)
        assertNull(processor.worldMagneticModel)
        assertFalse(processor.useWorldMagneticModel)
        assertTrue(processor.useAccurateRelativeGyroscopeAttitudeProcessor)
        assertTrue(processor.useIndirectInterpolation)
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            processor.interpolationValue,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            processor.indirectInterpolationWeight,
            0.0
        )
        assertEquals(0.0, processor.gyroscopeTimeIntervalSeconds, 0.0)
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            processor.outlierThreshold,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            processor.outlierPanicThreshold,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            processor.panicCounterThreshold
        )
    }

    @Test
    fun constructor_whenListener_returnsExpectedValues() {
        val listener =
            mockk<BaseFusedGeomagneticAttitudeProcessor.OnProcessedListener<AccelerometerSensorMeasurement, AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement>>()
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor(listener)

        assertSame(listener, processor.processorListener)
        assertEquals(Quaternion(), processor.fusedAttitude)
        assertEquals(0.0, processor.gx, 0.0)
        assertEquals(0.0, processor.gy, 0.0)
        assertEquals(0.0, processor.gz, 0.0)
        val gravity1 = processor.gravity
        assertEquals(AccelerationTriad(), gravity1)
        val gravity2 = AccelerationTriad()
        processor.getGravity(gravity2)
        assertEquals(gravity1, gravity2)
        assertNull(processor.location)
        assertTrue(processor.adjustGravityNorm)
        assertNull(processor.currentDate)
        assertFalse(processor.useAccurateLevelingProcessor)
        assertNull(processor.worldMagneticModel)
        assertFalse(processor.useWorldMagneticModel)
        assertTrue(processor.useAccurateRelativeGyroscopeAttitudeProcessor)
        assertTrue(processor.useIndirectInterpolation)
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            processor.interpolationValue,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            processor.indirectInterpolationWeight,
            0.0
        )
        assertEquals(0.0, processor.gyroscopeTimeIntervalSeconds, 0.0)
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            processor.outlierThreshold,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            processor.outlierPanicThreshold,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            processor.panicCounterThreshold
        )
    }

    @Test
    fun processorListener_setsExpectedValue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // check default value
        assertNull(processor.processorListener)

        val listener =
            mockk<BaseFusedGeomagneticAttitudeProcessor.OnProcessedListener<AccelerometerSensorMeasurement, AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement>>()
        processor.processorListener = listener

        // check
        assertSame(listener, processor.processorListener)
    }

    @Test
    fun gx_returnsExpectedValue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // setup spy
        val geomagneticProcessor: BaseGeomagneticAttitudeProcessor<GravitySensorMeasurement, *>? =
            processor.getPrivateProperty("geomagneticProcessor")
        requireNotNull(geomagneticProcessor)
        val geomagneticProcessorSpy = spyk(geomagneticProcessor)
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextDouble()
        every { geomagneticProcessorSpy.gx }.returns(gx)
        processor.setPrivateProperty("geomagneticProcessor", geomagneticProcessorSpy)

        assertEquals(gx, processor.gx, 0.0)
        verify(exactly = 1) { geomagneticProcessorSpy.gx }
    }

    @Test
    fun gy_returnsExpectedValue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // setup spy
        val geomagneticProcessor: BaseGeomagneticAttitudeProcessor<GravitySensorMeasurement, *>? =
            processor.getPrivateProperty("geomagneticProcessor")
        requireNotNull(geomagneticProcessor)
        val geomagneticProcessorSpy = spyk(geomagneticProcessor)
        val randomizer = UniformRandomizer()
        val gy = randomizer.nextDouble()
        every { geomagneticProcessorSpy.gy }.returns(gy)
        processor.setPrivateProperty("geomagneticProcessor", geomagneticProcessorSpy)

        assertEquals(gy, processor.gy, 0.0)
        verify(exactly = 1) { geomagneticProcessorSpy.gy }
    }

    @Test
    fun gz_returnsExpectedValue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // setup spy
        val geomagneticProcessor: BaseGeomagneticAttitudeProcessor<GravitySensorMeasurement, *>? =
            processor.getPrivateProperty("geomagneticProcessor")
        requireNotNull(geomagneticProcessor)
        val geomagneticProcessorSpy = spyk(geomagneticProcessor)
        val randomizer = UniformRandomizer()
        val gz = randomizer.nextDouble()
        every { geomagneticProcessorSpy.gz }.returns(gz)
        processor.setPrivateProperty("geomagneticProcessor", geomagneticProcessorSpy)

        assertEquals(gz, processor.gz, 0.0)
        verify(exactly = 1) { geomagneticProcessorSpy.gz }
    }

    @Test
    fun gravity_returnsExpectedValue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // setup spy
        val geomagneticProcessor: BaseGeomagneticAttitudeProcessor<GravitySensorMeasurement, *>? =
            processor.getPrivateProperty("geomagneticProcessor")
        requireNotNull(geomagneticProcessor)
        val geomagneticProcessorSpy = spyk(geomagneticProcessor)
        val gravity = AccelerationTriad()
        every { geomagneticProcessorSpy.gravity }.returns(gravity)
        processor.setPrivateProperty("geomagneticProcessor", geomagneticProcessorSpy)

        assertSame(gravity, processor.gravity)
        verify(exactly = 1) { geomagneticProcessorSpy.gravity }
    }

    @Test
    fun getGravity_returnsExpectedValue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // setup spy
        val geomagneticProcessor: BaseGeomagneticAttitudeProcessor<GravitySensorMeasurement, *>? =
            processor.getPrivateProperty("geomagneticProcessor")
        requireNotNull(geomagneticProcessor)
        val geomagneticProcessorSpy = spyk(geomagneticProcessor)
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextDouble()
        val gy = randomizer.nextDouble()
        val gz = randomizer.nextDouble()
        every { geomagneticProcessorSpy.getGravity(any()) }.answers { answer ->
            val triad = answer.invocation.args[0] as AccelerationTriad
            triad.setValueCoordinatesAndUnit(gx, gy, gz, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        }
        processor.setPrivateProperty("geomagneticProcessor", geomagneticProcessorSpy)

        val gravity = AccelerationTriad()
        processor.getGravity(gravity)

        // check
        assertEquals(gx, gravity.valueX, 0.0)
        assertEquals(gy, gravity.valueY, 0.0)
        assertEquals(gz, gravity.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, gravity.unit)

        verify(exactly = 1) { geomagneticProcessorSpy.getGravity(gravity) }
    }

    @Test
    fun location_setsExpectedValue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // check default value
        assertNull(processor.location)

        // setup spy
        val geomagneticProcessor: BaseGeomagneticAttitudeProcessor<GravitySensorMeasurement, *>? =
            processor.getPrivateProperty("geomagneticProcessor")
        requireNotNull(geomagneticProcessor)
        val geomagneticProcessorSpy = spyk(geomagneticProcessor)
        processor.setPrivateProperty("geomagneticProcessor", geomagneticProcessorSpy)

        // set new value
        val location = getLocation()
        processor.location = location

        // check
        assertSame(location, processor.location)

        verify(exactly = 1) { geomagneticProcessorSpy.location }
        verify(exactly = 1) { geomagneticProcessorSpy.location = location }

        assertSame(location, geomagneticProcessorSpy.location)
    }

    @Test
    fun adjustGravityNorm_setsExpectedValue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        val geomagneticProcessor: BaseGeomagneticAttitudeProcessor<GravitySensorMeasurement, *>? =
            processor.getPrivateProperty("geomagneticProcessor")
        requireNotNull(geomagneticProcessor)

        // check default value
        assertTrue(processor.adjustGravityNorm)
        assertTrue(geomagneticProcessor.adjustGravityNorm)

        // set new value
        processor.adjustGravityNorm = false

        // check
        assertFalse(processor.adjustGravityNorm)
        assertFalse(geomagneticProcessor.adjustGravityNorm)
    }

    @Test
    fun currentDate_setsExpectedValue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // check default value
        assertNull(processor.currentDate)

        // setup spy
        val geomagneticProcessor: BaseGeomagneticAttitudeProcessor<GravitySensorMeasurement, *>? =
            processor.getPrivateProperty("geomagneticProcessor")
        requireNotNull(geomagneticProcessor)
        val geomagneticProcessorSpy = spyk(geomagneticProcessor)
        processor.setPrivateProperty("geomagneticProcessor", geomagneticProcessorSpy)

        // set new value
        val currentDate = Date()
        processor.currentDate = currentDate

        // check
        assertSame(currentDate, processor.currentDate)

        verify(exactly = 1) { geomagneticProcessorSpy.currentDate }
        verify(exactly = 1) { geomagneticProcessorSpy.currentDate = currentDate }
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateLevelingProcessor_whenTrueAndLocationUnavailable_throwsIllegalStateException() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // check default value
        assertNull(processor.location)
        assertFalse(processor.useAccurateLevelingProcessor)

        // set new value
        processor.useAccurateLevelingProcessor = true
    }

    @Test
    fun useAccurateLevelingProcessor_whenValid_setsExpectedValue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // check default value
        assertNull(processor.location)
        assertFalse(processor.useAccurateLevelingProcessor)

        // setup spy
        val geomagneticProcessor: BaseGeomagneticAttitudeProcessor<GravitySensorMeasurement, *>? =
            processor.getPrivateProperty("geomagneticProcessor")
        requireNotNull(geomagneticProcessor)
        val geomagneticProcessorSpy = spyk(geomagneticProcessor)
        processor.setPrivateProperty("geomagneticProcessor", geomagneticProcessorSpy)

        // set location
        val location = getLocation()
        processor.location = location

        // set to true
        processor.useAccurateLevelingProcessor = true

        // check
        assertSame(location, processor.location)
        assertTrue(processor.useAccurateLevelingProcessor)

        verify(exactly = 1) { geomagneticProcessorSpy.useAccurateLevelingProcessor }
        verify(exactly = 1) { geomagneticProcessorSpy.useAccurateLevelingProcessor = true }
    }

    @Test
    fun worldMagneticModel_setsExpectedValue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // check default value
        assertNull(processor.worldMagneticModel)

        // setup spy
        val geomagneticProcessor: BaseGeomagneticAttitudeProcessor<GravitySensorMeasurement, *>? =
            processor.getPrivateProperty("geomagneticProcessor")
        requireNotNull(geomagneticProcessor)
        val geomagneticProcessorSpy = spyk(geomagneticProcessor)
        processor.setPrivateProperty("geomagneticProcessor", geomagneticProcessorSpy)

        // set new value
        val model = WorldMagneticModel()
        processor.worldMagneticModel = model

        // check
        assertSame(model, processor.worldMagneticModel)

        verify(exactly = 1) { geomagneticProcessorSpy.worldMagneticModel }
        verify(exactly = 1) { geomagneticProcessorSpy.worldMagneticModel = model }
    }

    @Test
    fun useWorldMagneticModel_setsExpectedValue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // check default value
        assertFalse(processor.useWorldMagneticModel)

        // setup spy
        val geomagneticProcessor: BaseGeomagneticAttitudeProcessor<GravitySensorMeasurement, *>? =
            processor.getPrivateProperty("geomagneticProcessor")
        requireNotNull(geomagneticProcessor)
        val geomagneticProcessorSpy = spyk(geomagneticProcessor)
        processor.setPrivateProperty("geomagneticProcessor", geomagneticProcessorSpy)

        // set new value
        processor.useWorldMagneticModel = true

        // check
        assertTrue(processor.useWorldMagneticModel)

        verify(exactly = 1) { geomagneticProcessorSpy.useWorldMagneticModel }
        verify(exactly = 1) { geomagneticProcessorSpy.useWorldMagneticModel = true }
    }

    @Test
    fun useAccurateRelativeGyroscopeAttitudeProcessor_whenTrue_buildsRelativeGyroscopeProcessor() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // check default value
        assertTrue(processor.useAccurateRelativeGyroscopeAttitudeProcessor)

        val relativeGyroscopeProcessor1: BaseRelativeGyroscopeAttitudeProcessor? =
            getPrivateProperty(
                BaseFusedGeomagneticAttitudeProcessor::class,
                processor,
                "relativeGyroscopeProcessor"
            )
        requireNotNull(relativeGyroscopeProcessor1)
        assertTrue(relativeGyroscopeProcessor1 is AccurateRelativeGyroscopeAttitudeProcessor)

        // set new value
        processor.useAccurateRelativeGyroscopeAttitudeProcessor = false

        // check
        assertFalse(processor.useAccurateRelativeGyroscopeAttitudeProcessor)

        val relativeGyroscopeProcessor2: BaseRelativeGyroscopeAttitudeProcessor? =
            getPrivateProperty(
                BaseFusedGeomagneticAttitudeProcessor::class,
                processor,
                "relativeGyroscopeProcessor"
            )
        requireNotNull(relativeGyroscopeProcessor2)
        assertTrue(relativeGyroscopeProcessor2 is RelativeGyroscopeAttitudeProcessor)
    }

    @Test
    fun useIndirectInterpolation_setsExpectedValue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // check default value
        assertTrue(processor.useIndirectInterpolation)

        // set new value
        processor.useIndirectInterpolation = false

        // check
        @Suppress("KotlinConstantConditions")
        assertFalse(processor.useIndirectInterpolation)
    }

    @Test
    fun interpolationValue_whenOutOfRange_throwsIllegalArgumentException() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            processor.interpolationValue,
            0.0
        )

        assertThrows(IllegalArgumentException::class.java) {
            processor.interpolationValue = -1.0
        }
        assertThrows(IllegalArgumentException::class.java) {
            processor.interpolationValue = 2.0
        }
    }

    @Test
    fun interpolationValue_whenValid_setsExpectedValue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            processor.interpolationValue,
            0.0
        )

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        processor.interpolationValue = value

        // check
        assertEquals(value, processor.interpolationValue, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun indirectInterpolationWeight_whenInvalid_throwsIllegalArgumentException() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        processor.indirectInterpolationWeight = 0.0
    }

    @Test
    fun indirectInterpolationWeight_whenValid_setsExpectedValue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // check default value
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            processor.indirectInterpolationWeight,
            0.0
        )

        // sets new value
        val randomizer = UniformRandomizer()
        val indirectInterpolationWeight = randomizer.nextDouble()
        processor.indirectInterpolationWeight = indirectInterpolationWeight

        // check
        assertEquals(indirectInterpolationWeight, processor.indirectInterpolationWeight, 0.0)
    }

    @Test
    fun gyroscopeTimeIntervalSeconds_returnsExpectedValue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // check default value
        assertEquals(0.0, processor.gyroscopeTimeIntervalSeconds, 0.0)

        // setup spy
        val relativeGyroscopeProcessor: BaseRelativeGyroscopeAttitudeProcessor? =
            getPrivateProperty(
                BaseFusedGeomagneticAttitudeProcessor::class,
                processor,
                "relativeGyroscopeProcessor"
            )
        requireNotNull(relativeGyroscopeProcessor)
        val relativeGyroscopeProcessorSpy = spyk(relativeGyroscopeProcessor)
        val randomizer = UniformRandomizer()
        val timeIntervalSeconds = randomizer.nextDouble()
        every { relativeGyroscopeProcessorSpy.timeIntervalSeconds }.returns(timeIntervalSeconds)
        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "relativeGyroscopeProcessor",
            relativeGyroscopeProcessorSpy
        )

        // check
        assertEquals(timeIntervalSeconds, processor.gyroscopeTimeIntervalSeconds, 0.0)
        verify(exactly = 1) { relativeGyroscopeProcessorSpy.timeIntervalSeconds }
    }

    @Test
    fun outlierThreshold_whenOutOfRange_throwsIllegalArgumentException() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // check default value
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            processor.outlierThreshold,
            0.0
        )

        // set invalid values
        assertThrows(IllegalArgumentException::class.java) {
            processor.outlierThreshold = -1.0
        }
        assertThrows(IllegalArgumentException::class.java) {
            processor.outlierThreshold = 2.0
        }
    }

    @Test
    fun outlierThreshold_whenValid_setsExpectedValue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // check default value
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            processor.outlierThreshold,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val outlierThreshold = randomizer.nextDouble()

        processor.outlierThreshold = outlierThreshold

        // check
        assertEquals(outlierThreshold, processor.outlierThreshold, 0.0)
    }

    @Test
    fun outlierPanicThreshold_whenOutOfRange_throwsIllegalArgumentException() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // check default value
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            processor.outlierPanicThreshold,
            0.0
        )

        // set invalid values
        assertThrows(IllegalArgumentException::class.java) {
            processor.outlierPanicThreshold = -1.0
        }
        assertThrows(IllegalArgumentException::class.java) {
            processor.outlierPanicThreshold = 2.0
        }
    }

    @Test
    fun outlierPanicThreshold_whenValid_setsExpectedValue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // check default value
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            processor.outlierPanicThreshold,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val outlierPanicThreshold = randomizer.nextDouble()
        processor.outlierPanicThreshold = outlierPanicThreshold

        // check
        assertEquals(outlierPanicThreshold, processor.outlierPanicThreshold, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun panicCounterThreshold_whenInvalid_throwsIllegalArgumentException() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        processor.panicCounterThreshold = 0
    }

    @Test
    fun panicCounterThreshold_whenValid_setsExpectedValue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // check default value
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            processor.panicCounterThreshold
        )

        // set new value
        processor.panicCounterThreshold = 2

        // check
        assertEquals(2, processor.panicCounterThreshold)
    }

    @Test
    fun reset_resetsInternalPropertiesAndProcessors() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // setup spies and properties
        val geomagneticProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            processor.getPrivateProperty("geomagneticProcessor")
        requireNotNull(geomagneticProcessor)
        val geomagneticProcessorSpy = spyk(geomagneticProcessor)
        processor.setPrivateProperty("geomagneticProcessor", geomagneticProcessorSpy)

        val relativeGyroscopeProcessor: BaseRelativeGyroscopeAttitudeProcessor? =
            getPrivateProperty(
                BaseFusedGeomagneticAttitudeProcessor::class,
                processor,
                "relativeGyroscopeProcessor"
            )
        requireNotNull(relativeGyroscopeProcessor)
        val relativeGyroscopeProcessorSpy = spyk(relativeGyroscopeProcessor)
        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "relativeGyroscopeProcessor",
            relativeGyroscopeProcessorSpy
        )

        val previousRelativeAttitude = getAttitude()
        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "previousRelativeAttitude",
            previousRelativeAttitude
        )
        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "hasDeltaRelativeAttitude",
            true
        )
        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "panicCounter",
            0
        )

        // check
        val previousRelativeAttitude1: Quaternion? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "previousRelativeAttitude"
        )
        requireNotNull(previousRelativeAttitude1)
        assertSame(previousRelativeAttitude, previousRelativeAttitude1)
        val hasDeltaRelativeAttitude1: Boolean? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "hasDeltaRelativeAttitude"
        )
        requireNotNull(hasDeltaRelativeAttitude1)
        assertTrue(hasDeltaRelativeAttitude1)
        val panicCounter1: Int? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "panicCounter"
        )
        requireNotNull(panicCounter1)
        assertEquals(0, panicCounter1)

        // reset
        processor.reset()

        // check
        val previousRelativeAttitude2: Quaternion? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "previousRelativeAttitude"
        )
        assertNull(previousRelativeAttitude2)
        val hasDeltaRelativeAttitude2: Boolean? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "hasDeltaRelativeAttitude"
        )
        requireNotNull(hasDeltaRelativeAttitude2)
        assertFalse(hasDeltaRelativeAttitude2)
        val panicCounter2: Int? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "panicCounter"
        )
        requireNotNull(panicCounter2)
        assertNotEquals(0, panicCounter2)
        assertEquals(processor.panicCounterThreshold, panicCounter2)

        verify(exactly = 1) { geomagneticProcessorSpy.reset() }
        verify(exactly = 1) { relativeGyroscopeProcessorSpy.reset() }
    }

    @Test
    fun process_whenEmpty_returnsFalse() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        val syncedMeasurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement()
        assertFalse(processor.process(syncedMeasurement))
    }

    @Test
    fun process_whenNoAccelerometerMeasurement_returnsFalse() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        val syncedMeasurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement(
            accelerometerMeasurement = null,
            GyroscopeSensorMeasurement(),
            MagnetometerSensorMeasurement(),
            0L
        )
        assertFalse(processor.process(syncedMeasurement))
    }

    @Test
    fun process_whenNoGyroscopeMeasurement_returnsFalse() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        val syncedMeasurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement(
            AccelerometerSensorMeasurement(),
            gyroscopeMeasurement = null,
            MagnetometerSensorMeasurement(),
            0L
        )
        assertFalse(processor.process(syncedMeasurement))
    }

    @Test
    fun process_whenNoMagnetometerMeasurement_returnsFalse() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        val syncedMeasurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement(
            AccelerometerSensorMeasurement(),
            GyroscopeSensorMeasurement(),
            magnetometerMeasurement = null,
            0L
        )
        assertFalse(processor.process(syncedMeasurement))
    }

    @Test
    fun process_whenGeomagneticAttitudeNotProcessed_returnsFalse() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // setup
        val geomagneticProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            processor.getPrivateProperty("geomagneticProcessor")
        requireNotNull(geomagneticProcessor)
        val geomagneticProcessorSpy = spyk(geomagneticProcessor)
        every { geomagneticProcessorSpy.process(any(), any()) }.returns(false)
        processor.setPrivateProperty("geomagneticProcessor", geomagneticProcessorSpy)

        val syncedMeasurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement(
            AccelerometerSensorMeasurement(),
            GyroscopeSensorMeasurement(),
            MagnetometerSensorMeasurement(),
            System.nanoTime()
        )

        assertFalse(processor.process(syncedMeasurement))
    }

    @Test
    fun process_whenRelativeAttitudeNotProcessed_returnsFalseAndKeepsGeomagneticAttitude() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // setup
        val geomagneticProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            processor.getPrivateProperty("geomagneticProcessor")
        requireNotNull(geomagneticProcessor)
        val geomagneticProcessorSpy = spyk(geomagneticProcessor)
        every { geomagneticProcessorSpy.process(any(), any()) }.returns(true)
        val geomagneticAttitude = getAttitude()
        geomagneticProcessorSpy.fusedAttitude.fromQuaternion(geomagneticAttitude)
        processor.setPrivateProperty("geomagneticProcessor", geomagneticProcessorSpy)

        val relativeGyroscopeProcessor: BaseRelativeGyroscopeAttitudeProcessor? =
            getPrivateProperty(
                BaseFusedGeomagneticAttitudeProcessor::class,
                processor,
                "relativeGyroscopeProcessor"
            )
        requireNotNull(relativeGyroscopeProcessor)
        val relativeGyroscopeProcessorSpy = spyk(relativeGyroscopeProcessor)
        every { relativeGyroscopeProcessorSpy.process(any(), any()) }.returns(false)
        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "relativeGyroscopeProcessor",
            relativeGyroscopeProcessorSpy
        )

        // check
        val geomagneticAttitude1: Quaternion? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "geomagneticAttitude"
        )
        requireNotNull(geomagneticAttitude1)
        assertEquals(Quaternion(), geomagneticAttitude1)

        // process
        val syncedMeasurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement(
            AccelerometerSensorMeasurement(),
            GyroscopeSensorMeasurement(),
            MagnetometerSensorMeasurement(),
            System.nanoTime()
        )

        assertFalse(processor.process(syncedMeasurement))

        // check
        val geomagneticAttitude2: Quaternion? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "geomagneticAttitude"
        )
        requireNotNull(geomagneticAttitude2)
        assertEquals(geomagneticAttitude, geomagneticAttitude2)
        assertSame(geomagneticAttitude1, geomagneticAttitude2)
    }

    @Test
    fun process_whenRelativeAttitudeProcessedAndNoDeltaRelativeAttitude_returnsFalse() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // setup
        val geomagneticProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            processor.getPrivateProperty("geomagneticProcessor")
        requireNotNull(geomagneticProcessor)
        val geomagneticProcessorSpy = spyk(geomagneticProcessor)
        every { geomagneticProcessorSpy.process(any(), any()) }.returns(true)
        val geomagneticAttitude = getAttitude()
        geomagneticProcessorSpy.fusedAttitude.fromQuaternion(geomagneticAttitude)
        processor.setPrivateProperty("geomagneticProcessor", geomagneticProcessorSpy)

        val relativeGyroscopeProcessor: BaseRelativeGyroscopeAttitudeProcessor? =
            getPrivateProperty(
                BaseFusedGeomagneticAttitudeProcessor::class,
                processor,
                "relativeGyroscopeProcessor"
            )
        requireNotNull(relativeGyroscopeProcessor)
        val relativeGyroscopeProcessorSpy = spyk(relativeGyroscopeProcessor)
        every { relativeGyroscopeProcessorSpy.process(any(), any()) }.returns(true)
        val relativeAttitude = getAttitude()
        relativeGyroscopeProcessorSpy.attitude.fromQuaternion(relativeAttitude)
        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "relativeGyroscopeProcessor",
            relativeGyroscopeProcessorSpy
        )

        // process
        val syncedMeasurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement(
            AccelerometerSensorMeasurement(),
            GyroscopeSensorMeasurement(),
            MagnetometerSensorMeasurement(),
            System.nanoTime()
        )

        // check
        assertFalse(processor.process(syncedMeasurement))

        val relativeAttitude1: Quaternion? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "relativeAttitude"
        )
        requireNotNull(relativeAttitude1)
        assertEquals(relativeAttitude, relativeAttitude1)

        val previousRelativeAttitude: Quaternion? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "previousRelativeAttitude"
        )
        requireNotNull(previousRelativeAttitude)
        assertEquals(relativeAttitude, previousRelativeAttitude)

        val hasDeltaRelativeAttitude: Boolean? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "hasDeltaRelativeAttitude"
        )
        requireNotNull(hasDeltaRelativeAttitude)
        assertFalse(hasDeltaRelativeAttitude)
    }

    @Test
    fun process_whenRelativeAttitudeProcessedAndDeltaRelativeAttitude_returnsTrue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // setup
        val geomagneticProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            processor.getPrivateProperty("geomagneticProcessor")
        requireNotNull(geomagneticProcessor)
        val geomagneticProcessorSpy = spyk(geomagneticProcessor)
        every { geomagneticProcessorSpy.process(any(), any()) }.returns(true)
        val geomagneticAttitude = getAttitude()
        geomagneticProcessorSpy.fusedAttitude.fromQuaternion(geomagneticAttitude)
        processor.setPrivateProperty("geomagneticProcessor", geomagneticProcessorSpy)

        val relativeGyroscopeProcessor: BaseRelativeGyroscopeAttitudeProcessor? =
            getPrivateProperty(
                BaseFusedGeomagneticAttitudeProcessor::class,
                processor,
                "relativeGyroscopeProcessor"
            )
        requireNotNull(relativeGyroscopeProcessor)
        val relativeGyroscopeProcessorSpy = spyk(relativeGyroscopeProcessor)
        every { relativeGyroscopeProcessorSpy.process(any(), any()) }.returns(true)
        val relativeAttitude = getAttitude()
        relativeGyroscopeProcessorSpy.attitude.fromQuaternion(relativeAttitude)
        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "relativeGyroscopeProcessor",
            relativeGyroscopeProcessorSpy
        )

        val previousRelativeAttitude = getAttitude()
        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "previousRelativeAttitude",
            previousRelativeAttitude
        )

        // process
        val syncedMeasurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement(
            AccelerometerSensorMeasurement(),
            GyroscopeSensorMeasurement(),
            MagnetometerSensorMeasurement(),
            System.nanoTime()
        )

        // check
        assertTrue(processor.process(syncedMeasurement))

        val relativeAttitude1: Quaternion? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "relativeAttitude"
        )
        requireNotNull(relativeAttitude1)
        assertEquals(relativeAttitude, relativeAttitude1)

        val deltaRelativeAttitude1: Quaternion? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "deltaRelativeAttitude"
        )
        val deltaRelativeAttitude2 =
            relativeAttitude.multiplyAndReturnNew(previousRelativeAttitude.inverseAndReturnNew())
        assertEquals(deltaRelativeAttitude2, deltaRelativeAttitude1)

        val hasDeltaRelativeAttitude: Boolean? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "hasDeltaRelativeAttitude"
        )
        requireNotNull(hasDeltaRelativeAttitude)
        assertTrue(hasDeltaRelativeAttitude)

        val internalFusedAttitude: Quaternion? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "internalFusedAttitude"
        )
        requireNotNull(internalFusedAttitude)
        assertEquals(geomagneticAttitude, internalFusedAttitude)
        assertEquals(internalFusedAttitude, processor.fusedAttitude)
    }

    @Test
    fun process_whenRelativeAttitudeProcessedNoResetToGeomagneticAndMediumDivergence_returnsTrue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // setup
        val geomagneticProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            processor.getPrivateProperty("geomagneticProcessor")
        requireNotNull(geomagneticProcessor)
        val geomagneticProcessorSpy = spyk(geomagneticProcessor)
        every { geomagneticProcessorSpy.process(any(), any()) }.returns(true)
        val geomagneticAttitude = getAttitude()
        geomagneticProcessorSpy.fusedAttitude.fromQuaternion(geomagneticAttitude)
        processor.setPrivateProperty("geomagneticProcessor", geomagneticProcessorSpy)

        val relativeGyroscopeProcessor: BaseRelativeGyroscopeAttitudeProcessor? =
            getPrivateProperty(
                BaseFusedGeomagneticAttitudeProcessor::class,
                processor,
                "relativeGyroscopeProcessor"
            )
        requireNotNull(relativeGyroscopeProcessor)
        val relativeGyroscopeProcessorSpy = spyk(relativeGyroscopeProcessor)
        every { relativeGyroscopeProcessorSpy.process(any(), any()) }.returns(true)
        val relativeAttitude = getAttitude()
        relativeGyroscopeProcessorSpy.attitude.fromQuaternion(relativeAttitude)
        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "relativeGyroscopeProcessor",
            relativeGyroscopeProcessorSpy
        )

        val previousRelativeAttitude = getAttitude()
        val previousRelativeAttitudeCopy = Quaternion(previousRelativeAttitude)
        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "previousRelativeAttitude",
            previousRelativeAttitude
        )

        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "panicCounter",
            0
        )

        val internalFusedAttitude = getAttitude()
        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "internalFusedAttitude",
            internalFusedAttitude
        )
        val internalFusedAttitudeCopy = Quaternion(internalFusedAttitude)

        mockkObject(QuaternionHelper)
        every { QuaternionHelper.dotProduct(any(), any()) }.returns(processor.outlierPanicThreshold)


        // process
        val syncedMeasurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement(
            AccelerometerSensorMeasurement(),
            GyroscopeSensorMeasurement(),
            MagnetometerSensorMeasurement(),
            System.nanoTime()
        )

        // check
        assertTrue(processor.process(syncedMeasurement))

        val relativeAttitude1: Quaternion? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "relativeAttitude"
        )
        requireNotNull(relativeAttitude1)
        assertEquals(relativeAttitude, relativeAttitude1)

        val deltaRelativeAttitude1: Quaternion? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "deltaRelativeAttitude"
        )
        val deltaRelativeAttitude2 =
            relativeAttitude.multiplyAndReturnNew(previousRelativeAttitudeCopy.inverseAndReturnNew())
        assertEquals(deltaRelativeAttitude2, deltaRelativeAttitude1)

        val hasDeltaRelativeAttitude: Boolean? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "hasDeltaRelativeAttitude"
        )
        requireNotNull(hasDeltaRelativeAttitude)
        assertTrue(hasDeltaRelativeAttitude)

        val internalFusedAttitude1: Quaternion? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "internalFusedAttitude"
        )
        requireNotNull(internalFusedAttitude1)
        val internalFusedAttitude2 =
            deltaRelativeAttitude2.multiplyAndReturnNew(internalFusedAttitudeCopy)
        assertEquals(internalFusedAttitude2, internalFusedAttitude1)
        assertEquals(internalFusedAttitude2, processor.fusedAttitude)

        assertEquals(relativeAttitude, previousRelativeAttitude)

        val panicCounter: Int? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "panicCounter"
        )
        requireNotNull(panicCounter)
        assertEquals(0, panicCounter)
    }

    @Test
    fun process_whenRelativeAttitudeProcessedNoResetToGeomagneticAndLargeDivergence_returnsTrue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()

        // setup
        val geomagneticProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            processor.getPrivateProperty("geomagneticProcessor")
        requireNotNull(geomagneticProcessor)
        val geomagneticProcessorSpy = spyk(geomagneticProcessor)
        every { geomagneticProcessorSpy.process(any(), any()) }.returns(true)
        val geomagneticAttitude = getAttitude()
        geomagneticProcessorSpy.fusedAttitude.fromQuaternion(geomagneticAttitude)
        processor.setPrivateProperty("geomagneticProcessor", geomagneticProcessorSpy)

        val relativeGyroscopeProcessor: BaseRelativeGyroscopeAttitudeProcessor? =
            getPrivateProperty(
                BaseFusedGeomagneticAttitudeProcessor::class,
                processor,
                "relativeGyroscopeProcessor"
            )
        requireNotNull(relativeGyroscopeProcessor)
        val relativeGyroscopeProcessorSpy = spyk(relativeGyroscopeProcessor)
        every { relativeGyroscopeProcessorSpy.process(any(), any()) }.returns(true)
        val relativeAttitude = getAttitude()
        relativeGyroscopeProcessorSpy.attitude.fromQuaternion(relativeAttitude)
        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "relativeGyroscopeProcessor",
            relativeGyroscopeProcessorSpy
        )

        val previousRelativeAttitude = getAttitude()
        val previousRelativeAttitudeCopy = Quaternion(previousRelativeAttitude)
        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "previousRelativeAttitude",
            previousRelativeAttitude
        )

        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "panicCounter",
            0
        )

        val internalFusedAttitude = getAttitude()
        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "internalFusedAttitude",
            internalFusedAttitude
        )
        val internalFusedAttitudeCopy = Quaternion(internalFusedAttitude)

        mockkObject(QuaternionHelper)
        every { QuaternionHelper.dotProduct(any(), any()) }.returns(0.0)


        // process
        val syncedMeasurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement(
            AccelerometerSensorMeasurement(),
            GyroscopeSensorMeasurement(),
            MagnetometerSensorMeasurement(),
            System.nanoTime()
        )

        // check
        assertTrue(processor.process(syncedMeasurement))

        val relativeAttitude1: Quaternion? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "relativeAttitude"
        )
        requireNotNull(relativeAttitude1)
        assertEquals(relativeAttitude, relativeAttitude1)

        val deltaRelativeAttitude1: Quaternion? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "deltaRelativeAttitude"
        )
        val deltaRelativeAttitude2 =
            relativeAttitude.multiplyAndReturnNew(previousRelativeAttitudeCopy.inverseAndReturnNew())
        assertEquals(deltaRelativeAttitude2, deltaRelativeAttitude1)

        val hasDeltaRelativeAttitude: Boolean? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "hasDeltaRelativeAttitude"
        )
        requireNotNull(hasDeltaRelativeAttitude)
        assertTrue(hasDeltaRelativeAttitude)

        val internalFusedAttitude1: Quaternion? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "internalFusedAttitude"
        )
        requireNotNull(internalFusedAttitude1)
        val internalFusedAttitude2 =
            deltaRelativeAttitude2.multiplyAndReturnNew(internalFusedAttitudeCopy)
        assertEquals(internalFusedAttitude2, internalFusedAttitude1)
        assertEquals(internalFusedAttitude2, processor.fusedAttitude)

        assertEquals(relativeAttitude, previousRelativeAttitude)

        val panicCounter: Int? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "panicCounter"
        )
        requireNotNull(panicCounter)
        assertEquals(1, panicCounter)
    }

    @Test
    fun process_whenRelativeAttitudeProcessedNoResetToGeomagneticSmallDivergenceAndDirectInterpolation_returnsTrue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()
        processor.useIndirectInterpolation = false

        // setup
        val geomagneticProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            processor.getPrivateProperty("geomagneticProcessor")
        requireNotNull(geomagneticProcessor)
        val geomagneticProcessorSpy = spyk(geomagneticProcessor)
        every { geomagneticProcessorSpy.process(any(), any()) }.returns(true)
        val geomagneticAttitude = getAttitude()
        geomagneticProcessorSpy.fusedAttitude.fromQuaternion(geomagneticAttitude)
        processor.setPrivateProperty("geomagneticProcessor", geomagneticProcessorSpy)

        val relativeGyroscopeProcessor: BaseRelativeGyroscopeAttitudeProcessor? =
            getPrivateProperty(
                BaseFusedGeomagneticAttitudeProcessor::class,
                processor,
                "relativeGyroscopeProcessor"
            )
        requireNotNull(relativeGyroscopeProcessor)
        val relativeGyroscopeProcessorSpy = spyk(relativeGyroscopeProcessor)
        every { relativeGyroscopeProcessorSpy.process(any(), any()) }.returns(true)
        val relativeAttitude = getAttitude()
        relativeGyroscopeProcessorSpy.attitude.fromQuaternion(relativeAttitude)
        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "relativeGyroscopeProcessor",
            relativeGyroscopeProcessorSpy
        )

        val previousRelativeAttitude = getAttitude()
        val previousRelativeAttitudeCopy = Quaternion(previousRelativeAttitude)
        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "previousRelativeAttitude",
            previousRelativeAttitude
        )

        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "panicCounter",
            0
        )

        val internalFusedAttitude = getAttitude()
        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "internalFusedAttitude",
            internalFusedAttitude
        )
        val internalFusedAttitudeCopy = Quaternion(internalFusedAttitude)

        mockkObject(QuaternionHelper)
        every { QuaternionHelper.dotProduct(any(), any()) }.returns(processor.outlierThreshold)


        // process
        val syncedMeasurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement(
            AccelerometerSensorMeasurement(),
            GyroscopeSensorMeasurement(),
            MagnetometerSensorMeasurement(),
            System.nanoTime()
        )

        // check
        assertTrue(processor.process(syncedMeasurement))

        val relativeAttitude1: Quaternion? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "relativeAttitude"
        )
        requireNotNull(relativeAttitude1)
        assertEquals(relativeAttitude, relativeAttitude1)

        val deltaRelativeAttitude1: Quaternion? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "deltaRelativeAttitude"
        )
        val deltaRelativeAttitude2 =
            relativeAttitude.multiplyAndReturnNew(previousRelativeAttitudeCopy.inverseAndReturnNew())
        assertEquals(deltaRelativeAttitude2, deltaRelativeAttitude1)

        val hasDeltaRelativeAttitude: Boolean? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "hasDeltaRelativeAttitude"
        )
        requireNotNull(hasDeltaRelativeAttitude)
        assertTrue(hasDeltaRelativeAttitude)

        val internalFusedAttitude1: Quaternion? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "internalFusedAttitude"
        )
        requireNotNull(internalFusedAttitude1)
        val internalFusedAttitude2 = Quaternion.slerpAndReturnNew(
            deltaRelativeAttitude2.multiplyAndReturnNew(internalFusedAttitudeCopy),
            geomagneticAttitude,
            processor.interpolationValue
        )
        assertEquals(internalFusedAttitude2, internalFusedAttitude1)
        assertEquals(internalFusedAttitude2, processor.fusedAttitude)

        assertEquals(relativeAttitude, previousRelativeAttitude)

        val panicCounter: Int? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "panicCounter"
        )
        requireNotNull(panicCounter)
        assertEquals(0, panicCounter)
    }

    @Test
    fun process_whenRelativeAttitudeProcessedNoResetToGeomagneticSmallDivergenceAndIndirectInterpolation_returnsTrue() {
        val processor = AccelerometerFusedGeomagneticAttitudeProcessor()
        processor.useIndirectInterpolation = true

        // setup
        val geomagneticProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            processor.getPrivateProperty("geomagneticProcessor")
        requireNotNull(geomagneticProcessor)
        val geomagneticProcessorSpy = spyk(geomagneticProcessor)
        every { geomagneticProcessorSpy.process(any(), any()) }.returns(true)
        val geomagneticAttitude = getAttitude()
        geomagneticProcessorSpy.fusedAttitude.fromQuaternion(geomagneticAttitude)
        processor.setPrivateProperty("geomagneticProcessor", geomagneticProcessorSpy)

        val relativeGyroscopeProcessor: BaseRelativeGyroscopeAttitudeProcessor? =
            getPrivateProperty(
                BaseFusedGeomagneticAttitudeProcessor::class,
                processor,
                "relativeGyroscopeProcessor"
            )
        requireNotNull(relativeGyroscopeProcessor)
        val relativeGyroscopeProcessorSpy = spyk(relativeGyroscopeProcessor)
        every { relativeGyroscopeProcessorSpy.process(any(), any()) }.returns(true)
        every { relativeGyroscopeProcessorSpy.timeIntervalSeconds }.returns(TIME_INTERVAL)
        val relativeAttitude = getAttitude()
        relativeGyroscopeProcessorSpy.attitude.fromQuaternion(relativeAttitude)
        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "relativeGyroscopeProcessor",
            relativeGyroscopeProcessorSpy
        )

        val previousRelativeAttitude = getAttitude()
        val previousRelativeAttitudeCopy = Quaternion(previousRelativeAttitude)
        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "previousRelativeAttitude",
            previousRelativeAttitude
        )

        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "panicCounter",
            0
        )

        val internalFusedAttitude = getAttitude()
        setPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "internalFusedAttitude",
            internalFusedAttitude
        )
        val internalFusedAttitudeCopy = Quaternion(internalFusedAttitude)

        mockkObject(QuaternionHelper)
        every { QuaternionHelper.dotProduct(any(), any()) }.returns(processor.outlierThreshold)


        // process
        val syncedMeasurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement(
            AccelerometerSensorMeasurement(),
            GyroscopeSensorMeasurement(),
            MagnetometerSensorMeasurement(),
            System.nanoTime()
        )

        // check
        assertTrue(processor.process(syncedMeasurement))

        val relativeAttitude1: Quaternion? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "relativeAttitude"
        )
        requireNotNull(relativeAttitude1)
        assertEquals(relativeAttitude, relativeAttitude1)

        val deltaRelativeAttitude1: Quaternion? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "deltaRelativeAttitude"
        )
        val deltaRelativeAttitude2 =
            relativeAttitude.multiplyAndReturnNew(previousRelativeAttitudeCopy.inverseAndReturnNew())
        assertEquals(deltaRelativeAttitude2, deltaRelativeAttitude1)

        val hasDeltaRelativeAttitude: Boolean? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "hasDeltaRelativeAttitude"
        )
        requireNotNull(hasDeltaRelativeAttitude)
        assertTrue(hasDeltaRelativeAttitude)

        val internalFusedAttitude1: Quaternion? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "internalFusedAttitude"
        )
        requireNotNull(internalFusedAttitude1)
        val t = processor.interpolationValue + processor.indirectInterpolationWeight * abs(
            deltaRelativeAttitude2.rotationAngle / TIME_INTERVAL
        )
        val internalFusedAttitude2 = Quaternion.slerpAndReturnNew(
            deltaRelativeAttitude2.multiplyAndReturnNew(internalFusedAttitudeCopy),
            geomagneticAttitude,
            t
        )
        assertEquals(internalFusedAttitude2, internalFusedAttitude1)
        assertEquals(internalFusedAttitude2, processor.fusedAttitude)

        assertEquals(relativeAttitude, previousRelativeAttitude)

        val panicCounter: Int? = getPrivateProperty(
            BaseFusedGeomagneticAttitudeProcessor::class,
            processor,
            "panicCounter"
        )
        requireNotNull(panicCounter)
        assertEquals(0, panicCounter)
    }

    private companion object {
        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 4000.0

        const val MIN_ANGLE_DEGREES = -45.0
        const val MAX_ANGLE_DEGREES = 45.0

        const val TIME_INTERVAL = 0.02

        fun getLocation(): Location {
            val randomizer = UniformRandomizer()
            val latitudeDegrees = randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES)
            val longitudeDegrees =
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES)
            val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

            val location = mockk<Location>()
            every { location.latitude }.returns(latitudeDegrees)
            every { location.longitude }.returns(longitudeDegrees)
            every { location.altitude }.returns(height)

            return location
        }

        fun getAttitude(): Quaternion {
            val randomizer = UniformRandomizer()
            val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))

            return Quaternion(roll, pitch, yaw)
        }
    }
}