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
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.AccelerationUnit
import io.mockk.every
import io.mockk.mockkObject
//import io.mockk.*
//import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
//import org.junit.After
import org.junit.Assert.*
//import org.junit.Ignore
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.mockito.Mock
import org.mockito.junit.MockitoJUnit
import org.mockito.junit.MockitoRule
import org.mockito.kotlin.any
import org.mockito.kotlin.doAnswer
import org.mockito.kotlin.doReturn
import org.mockito.kotlin.only
import org.mockito.kotlin.spy
import org.mockito.kotlin.times
import org.mockito.kotlin.verify
import org.mockito.kotlin.whenever
import org.robolectric.RobolectricTestRunner
import kotlin.math.abs
import kotlin.math.min

//@Ignore("Possible memory leak when running this test")
@RunWith(RobolectricTestRunner::class)
class AccelerometerLeveledRelativeAttitudeProcessorTest {

    @get:Rule
    val mockitoRule: MockitoRule = MockitoJUnit.rule()

    @get:Rule
    val mockkRule = MockKRule(this)

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var processorListener:
            BaseLeveledRelativeAttitudeProcessor.OnProcessedListener<AccelerometerSensorMeasurement, AccelerometerAndGyroscopeSyncedSensorMeasurement>

//    @MockK
    @Mock
    private lateinit var location: Location

    /*@After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
        System.gc()
    }*/

    @Test
    fun constructor_whenNoParameters_setsDefaultValues() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        // check
        assertNull(processor.processorListener)
        assertEquals(Quaternion(), processor.fusedAttitude)
        assertEquals(0.0, processor.gx, 0.0)
        assertEquals(0.0, processor.gy, 0.0)
        assertEquals(0.0, processor.gz, 0.0)

        val gravity1 = processor.gravity
        assertEquals(0.0, gravity1.valueX, 0.0)
        assertEquals(0.0, gravity1.valueY, 0.0)
        assertEquals(0.0, gravity1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, gravity1.unit)
        val gravity2 = AccelerationTriad()
        processor.getGravity(gravity2)
        assertEquals(gravity1, gravity2)

        assertNull(processor.location)
        assertTrue(processor.adjustGravityNorm)
        assertFalse(processor.useAccurateLevelingProcessor)
        assertTrue(processor.useAccurateRelativeGyroscopeAttitudeProcessor)
        assertTrue(processor.useIndirectInterpolation)
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            processor.interpolationValue,
            0.0
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            processor.indirectInterpolationWeight,
            0.0
        )
        assertEquals(0.0, processor.timeIntervalSeconds, 0.0)
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            processor.outlierThreshold,
            0.0
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            processor.outlierPanicThreshold,
            0.0
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            processor.panicCounterThreshold
        )
    }

    @Test
    fun constructor_whenListener_setsExpectedValues() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor(processorListener)

        // check
        assertSame(processorListener, processor.processorListener)
        assertEquals(Quaternion(), processor.fusedAttitude)
        assertEquals(0.0, processor.gx, 0.0)
        assertEquals(0.0, processor.gy, 0.0)
        assertEquals(0.0, processor.gz, 0.0)

        val gravity1 = processor.gravity
        assertEquals(0.0, gravity1.valueX, 0.0)
        assertEquals(0.0, gravity1.valueY, 0.0)
        assertEquals(0.0, gravity1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, gravity1.unit)
        val gravity2 = AccelerationTriad()
        processor.getGravity(gravity2)
        assertEquals(gravity1, gravity2)

        assertNull(processor.location)
        assertTrue(processor.adjustGravityNorm)
        assertFalse(processor.useAccurateLevelingProcessor)
        assertTrue(processor.useAccurateRelativeGyroscopeAttitudeProcessor)
        assertTrue(processor.useIndirectInterpolation)
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            processor.interpolationValue,
            0.0
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            processor.indirectInterpolationWeight,
            0.0
        )
        assertEquals(0.0, processor.timeIntervalSeconds, 0.0)
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            processor.outlierThreshold,
            0.0
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            processor.outlierPanicThreshold,
            0.0
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            processor.panicCounterThreshold
        )
    }

    @Test
    fun gx_returnsExpectedValue() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)

        val randomizer = UniformRandomizer()
        val gx = randomizer.nextDouble()
        doReturn(gx).whenever(gravityProcessorSpy).gx
//        every { gravityProcessorSpy.gx }.returns(gx)
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        assertEquals(gx, processor.gx, 0.0)
        verify(gravityProcessorSpy, only()).gx
//        verify(exactly = 1) { gravityProcessorSpy.gx }
    }

    @Test
    fun gy_returnsExpectedValue() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)

        val randomizer = UniformRandomizer()
        val gy = randomizer.nextDouble()
        doReturn(gy).whenever(gravityProcessorSpy).gy
//        every { gravityProcessorSpy.gy }.returns(gy)
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        assertEquals(gy, processor.gy, 0.0)
        verify(gravityProcessorSpy, only()).gy
//        verify(exactly = 1) { gravityProcessorSpy.gy }
    }

    @Test
    fun gz_returnsExpectedValue() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)

        val randomizer = UniformRandomizer()
        val gz = randomizer.nextDouble()
        doReturn(gz).whenever(gravityProcessorSpy).gz
//        every { gravityProcessorSpy.gz }.returns(gz)
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        assertEquals(gz, processor.gz, 0.0)
        verify(gravityProcessorSpy, only()).gz
//        verify(exactly = 1) { gravityProcessorSpy.gz }
    }

    @Test
    fun gravity_returnsExpectedValue() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)

        val randomizer = UniformRandomizer()
        val gx = randomizer.nextDouble()
        val gy = randomizer.nextDouble()
        val gz = randomizer.nextDouble()
        doReturn(AccelerationTriad(gx, gy, gz)).whenever(gravityProcessorSpy).gravity
//        every { gravityProcessorSpy.gravity }.returns(AccelerationTriad(gx, gy, gz))
        doAnswer { invocation ->
            val triad = invocation.getArgument<AccelerationTriad>(0)
            triad.setValueCoordinatesAndUnit(gx, gy, gz, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        }.whenever(gravityProcessorSpy).getGravity(any())
/*        every { gravityProcessorSpy.getGravity(any()) }.answers { call ->
            val triad = call.invocation.args[0] as AccelerationTriad
            triad.setValueCoordinatesAndUnit(gx, gy, gz, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        }*/
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val gravity1 = processor.gravity
        assertEquals(gx, gravity1.valueX, 0.0)
        assertEquals(gy, gravity1.valueY, 0.0)
        assertEquals(gz, gravity1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, gravity1.unit)
        verify(gravityProcessorSpy, only()).gravity
//        verify(exactly = 1) { gravityProcessorSpy.gravity }

        val gravity2 = AccelerationTriad()
        processor.getGravity(gravity2)

        assertEquals(gravity1, gravity2)
        verify(gravityProcessorSpy, times(1)).getGravity(gravity2)
//        verify(exactly = 1) { gravityProcessorSpy.getGravity(gravity2) }
    }

    @Test
    fun location_setsExpectedValue() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        // check default value
        assertNull(processor.location)

        // set new value
        val location = getLocation()
        processor.location = location

        // check
        assertSame(location, processor.location)

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        assertSame(location, gravityProcessor.location)
    }

    @Test
    fun adjustGravityNorm_setsExpectedValue() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        // check default value
        assertTrue(processor.adjustGravityNorm)
        assertTrue(gravityProcessor.adjustGravityNorm)

        // set new value
        processor.adjustGravityNorm = false

        // check
        assertFalse(processor.adjustGravityNorm)
        assertFalse(gravityProcessor.adjustGravityNorm)
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateLevelingProcessor_whenTrueAndNoLocation_throwsIllegalStateException() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        // check default value
        assertFalse(processor.useAccurateLevelingProcessor)
        assertNull(processor.location)

        val levelingProcessor: BaseLevelingProcessor? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "levelingProcessor"
        )
        requireNotNull(levelingProcessor)
        assertTrue(levelingProcessor is LevelingProcessor)

        // set new value
        processor.useAccurateLevelingProcessor = true
    }

    @Test
    fun useAccurateLevelingProcessor_whenTrueAndLocation_setsValueAndRebuildsLevelingProcessor() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        // check default value
        assertFalse(processor.useAccurateLevelingProcessor)
        assertNull(processor.location)

        val levelingProcessor1: BaseLevelingProcessor? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "levelingProcessor"
        )
        requireNotNull(levelingProcessor1)
        assertTrue(levelingProcessor1 is LevelingProcessor)

        // set location
        val location = getLocation()
        processor.location = location

        // set new value
        processor.useAccurateLevelingProcessor = true

        // check
        assertTrue(processor.useAccurateLevelingProcessor)
        assertSame(location, processor.location)

        val levelingProcessor2: BaseLevelingProcessor? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "levelingProcessor"
        )
        requireNotNull(levelingProcessor2)
        assertTrue(levelingProcessor2 is AccurateLevelingProcessor)

        val accurateLevelingProcessor = levelingProcessor2 as AccurateLevelingProcessor
        assertSame(location, accurateLevelingProcessor.location)
    }

    @Test
    fun useAccurateRelativeGyroscopeAttitudeProcessor_setsValueAndRebuildsRelativeAttitudeProcessor() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        assertTrue(processor.useAccurateRelativeGyroscopeAttitudeProcessor)

        val relativeAttitudeProcessor1: BaseRelativeGyroscopeAttitudeProcessor? =
            getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "relativeAttitudeProcessor"
            )
        requireNotNull(relativeAttitudeProcessor1)
        assertTrue(relativeAttitudeProcessor1 is AccurateRelativeGyroscopeAttitudeProcessor)

        // set new value
        processor.useAccurateRelativeGyroscopeAttitudeProcessor = false

        // checks
        assertFalse(processor.useAccurateRelativeGyroscopeAttitudeProcessor)

        val relativeAttitudeProcessor2: BaseRelativeGyroscopeAttitudeProcessor? =
            getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "relativeAttitudeProcessor"
            )
        requireNotNull(relativeAttitudeProcessor2)
        assertTrue(relativeAttitudeProcessor2 is RelativeGyroscopeAttitudeProcessor)
    }

    @Suppress("KotlinConstantConditions")
    @Test
    fun useIndirectInterpolation_setsExpectedValue() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        assertTrue(processor.useIndirectInterpolation)

        // set new value
        processor.useIndirectInterpolation = false

        // check
        assertFalse(processor.useIndirectInterpolation)
    }

    @Test
    fun interpolationValue_whenOutOfRange_throwsIllegalArgumentException() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
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
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
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
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        processor.indirectInterpolationWeight = 0.0
    }

    @Test
    fun indirectInterpolationWeight_whenValid_setsExpectedValue() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        // check default value
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            processor.indirectInterpolationWeight,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val indirectInterpolationWeight = randomizer.nextDouble()
        processor.indirectInterpolationWeight = indirectInterpolationWeight

        // check
        assertEquals(indirectInterpolationWeight, processor.indirectInterpolationWeight, 0.0)
    }

    @Test
    fun gyroscopeAverageTimeInterval_returnsInternalAttitudeProcessorAverageTimeInterval() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        val relativeAttitudeProcessor: BaseRelativeGyroscopeAttitudeProcessor? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitudeProcessor"
        )
        requireNotNull(relativeAttitudeProcessor)
        val relativeAttitudeProcessorSpy = spy(relativeAttitudeProcessor)
//        val relativeAttitudeProcessorSpy = spyk(relativeAttitudeProcessor)
        doReturn(TIME_INTERVAL).whenever(relativeAttitudeProcessorSpy).timeIntervalSeconds
//        every { relativeAttitudeProcessorSpy.timeIntervalSeconds }.returns(TIME_INTERVAL)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitudeProcessor",
            relativeAttitudeProcessorSpy
        )

        assertEquals(TIME_INTERVAL, processor.timeIntervalSeconds, 0.0)
        verify(relativeAttitudeProcessorSpy, only()).timeIntervalSeconds
//        verify(exactly = 1) { relativeAttitudeProcessorSpy.timeIntervalSeconds }
    }

    @Test
    fun outlierThreshold_whenOutOfRange_throwsIllegalArgumentException() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        // check default value
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            processor.outlierThreshold,
            0.0
        )

        // set invalid value
        assertThrows(IllegalArgumentException::class.java) {
            processor.outlierThreshold = -1.0
        }
        assertThrows(IllegalArgumentException::class.java) {
            processor.outlierThreshold = 2.0
        }
    }

    @Test
    fun outlierThreshold_whenValid_setsExpectedValue() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        // check default value
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
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
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        // check default value
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
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
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        // check default value
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
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
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        // check default value
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            processor.panicCounterThreshold
        )

        // set invalid value
        processor.panicCounterThreshold = 0
    }

    @Test
    fun panicCounterThreshold_whenValid_setsExpectedValue() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        // check default value
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            processor.panicCounterThreshold
        )

        // set new value
        processor.panicCounterThreshold = 2

        // check
        assertEquals(2, processor.panicCounterThreshold)
    }

    @Test
    fun reset_restoresInitialState() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "panicCounter",
            processor.panicCounterThreshold + 1
        )

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val levelingProcessor: BaseLevelingProcessor? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "levelingProcessor"
        )
        requireNotNull(levelingProcessor)
        val levelingProcessorSpy = spy(levelingProcessor)
//        val levelingProcessorSpy = spyk(levelingProcessor)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "levelingProcessor",
            levelingProcessorSpy
        )

        val relativeAttitudeProcessor: BaseRelativeGyroscopeAttitudeProcessor? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitudeProcessor"
        )
        requireNotNull(relativeAttitudeProcessor)
        val relativeAttitudeProcessorSpy = spy(relativeAttitudeProcessor)
//        val relativeAttitudeProcessorSpy = spyk(relativeAttitudeProcessor)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitudeProcessor",
            relativeAttitudeProcessorSpy
        )

        // reset
        processor.reset()

        // check
        val panicCounter: Int? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "panicCounter"
        )
        requireNotNull(panicCounter)
        assertEquals(processor.panicCounterThreshold, panicCounter)

        verify(gravityProcessorSpy, only()).reset()
//        verify(exactly = 1) { gravityProcessorSpy.reset() }
        verify(levelingProcessorSpy, only()).reset()
//        verify(exactly = 1) { levelingProcessorSpy.reset() }
        verify(relativeAttitudeProcessorSpy, only()).reset()
//        verify(exactly = 1) { relativeAttitudeProcessorSpy.reset() }
    }

    @Test
    fun process_whenEmptySyncedMeasurement_returnsFalse() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        val syncedMeasurement = AccelerometerAndGyroscopeSyncedSensorMeasurement()
        assertFalse(processor.process(syncedMeasurement))
    }

    @Test
    fun process_whenMissingGyroscopeMeasurement_returnsFalse() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        val syncedMeasurement =
            AccelerometerAndGyroscopeSyncedSensorMeasurement(AccelerometerSensorMeasurement())
        assertFalse(processor.process(syncedMeasurement))
    }

    @Test
    fun process_whenRelativeAttitudeNotProcessed_returnsFalse() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        val accelerometerMeasurement = AccelerometerSensorMeasurement()
        val gyroscopeMeasurement = GyroscopeSensorMeasurement()
        val timestamp = System.nanoTime()
        val syncedMeasurement = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gyroscopeMeasurement,
            timestamp
        )
        assertFalse(processor.process(syncedMeasurement))
    }

    @Test
    fun process_whenRelativeAttitudeProcessedAndNoPreviousRelative_returnsFalse() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        // setup spies
        val relativeAttitudeProcessor: BaseRelativeGyroscopeAttitudeProcessor? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitudeProcessor"
        )
        requireNotNull(relativeAttitudeProcessor)
        val relativeAttitudeProcessorSpy = spy(relativeAttitudeProcessor)
//        val relativeAttitudeProcessorSpy = spyk(relativeAttitudeProcessor)
        doReturn(true).whenever(relativeAttitudeProcessorSpy).process(any(), any())
//        every { relativeAttitudeProcessorSpy.process(any(), any()) }.returns(true)
        val relativeAttitude = getAttitude()
        doReturn(relativeAttitude).whenever(relativeAttitudeProcessorSpy).attitude
//        every { relativeAttitudeProcessorSpy.attitude }.returns(relativeAttitude)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitudeProcessor",
            relativeAttitudeProcessorSpy
        )

        // check initial values
        val relativeAttitude1: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitude"
        )
        requireNotNull(relativeAttitude1)
        assertEquals(Quaternion(), relativeAttitude1)

        val previousRelativeAttitude1: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "previousRelativeAttitude"
        )
        assertNull(previousRelativeAttitude1)

        val timestamp1: Long? =
            getPrivateProperty(BaseLeveledRelativeAttitudeProcessor::class, processor, "timestamp")
        requireNotNull(timestamp1)
        assertEquals(0L, timestamp1)

        // process
        val accelerometerMeasurement = AccelerometerSensorMeasurement()
        val gyroscopeMeasurement = GyroscopeSensorMeasurement()
        val timestamp = System.nanoTime()
        val syncedMeasurement = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gyroscopeMeasurement,
            timestamp
        )
        assertFalse(processor.process(syncedMeasurement))

        // check
        val relativeAttitude2: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitude"
        )
        requireNotNull(relativeAttitude2)
        assertEquals(relativeAttitude, relativeAttitude2)

        val previousRelativeAttitude2: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "previousRelativeAttitude"
        )
        requireNotNull(previousRelativeAttitude2)
        assertEquals(relativeAttitude, previousRelativeAttitude2)

        val timestamp2: Long? =
            getPrivateProperty(BaseLeveledRelativeAttitudeProcessor::class, processor, "timestamp")
        requireNotNull(timestamp2)
        assertEquals(timestamp, timestamp2)

        verify(relativeAttitudeProcessorSpy, times(1)).process(
            gyroscopeMeasurement,
            timestamp
        )
/*        verify(exactly = 1) {
            relativeAttitudeProcessorSpy.process(
                gyroscopeMeasurement,
                timestamp
            )
        }*/
    }

    @Test
    fun process_whenRelativeAttitudeProcessedPreviousRelativeAndGravityNotProcessed_returnsFalse() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        // setup spies
        val relativeAttitudeProcessor: BaseRelativeGyroscopeAttitudeProcessor? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitudeProcessor"
        )
        requireNotNull(relativeAttitudeProcessor)
        val relativeAttitudeProcessorSpy = spy(relativeAttitudeProcessor)
//        val relativeAttitudeProcessorSpy = spyk(relativeAttitudeProcessor)
        doReturn(true).whenever(relativeAttitudeProcessorSpy).process(any(), any())
//        every { relativeAttitudeProcessorSpy.process(any(), any()) }.returns(true)
        val relativeAttitude = getAttitude()
        doReturn(relativeAttitude).whenever(relativeAttitudeProcessorSpy).attitude
//        every { relativeAttitudeProcessorSpy.attitude }.returns(relativeAttitude)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitudeProcessor",
            relativeAttitudeProcessorSpy
        )

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        doReturn(false).whenever(gravityProcessorSpy).process(any(), any())
//        every { gravityProcessorSpy.process(any(), any()) }.returns(false)
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val previousRelativeAttitude1 = getAttitude()
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "previousRelativeAttitude",
            previousRelativeAttitude1
        )

        // check initial values
        val relativeAttitude1: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitude"
        )
        requireNotNull(relativeAttitude1)
        assertEquals(Quaternion(), relativeAttitude1)

        val timestamp1: Long? =
            getPrivateProperty(BaseLeveledRelativeAttitudeProcessor::class, processor, "timestamp")
        requireNotNull(timestamp1)
        assertEquals(0L, timestamp1)

        // process
        val accelerometerMeasurement = AccelerometerSensorMeasurement()
        val gyroscopeMeasurement = GyroscopeSensorMeasurement()
        val timestamp = System.nanoTime()
        val syncedMeasurement = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gyroscopeMeasurement,
            timestamp
        )
        assertFalse(processor.process(syncedMeasurement))

        // check
        val relativeAttitude2: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitude"
        )
        requireNotNull(relativeAttitude2)
        assertEquals(relativeAttitude, relativeAttitude2)

        val previousRelativeAttitude2: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "previousRelativeAttitude"
        )
        requireNotNull(previousRelativeAttitude2)
        assertEquals(previousRelativeAttitude1, previousRelativeAttitude2)

        val inversePreviousRelativeAttitude: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "inversePreviousRelativeAttitude"
        )
        requireNotNull(inversePreviousRelativeAttitude)
        assertEquals(
            previousRelativeAttitude2.inverseAndReturnNew(),
            inversePreviousRelativeAttitude
        )

        val deltaRelativeAttitude: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "deltaRelativeAttitude"
        )
        requireNotNull(deltaRelativeAttitude)
        assertEquals(
            relativeAttitude2.multiplyAndReturnNew(inversePreviousRelativeAttitude),
            deltaRelativeAttitude
        )

        val timestamp2: Long? =
            getPrivateProperty(BaseLeveledRelativeAttitudeProcessor::class, processor, "timestamp")
        requireNotNull(timestamp2)
        assertEquals(timestamp, timestamp2)

        verify(relativeAttitudeProcessorSpy, times(1)).process(
            gyroscopeMeasurement,
            timestamp
        )
/*        verify(exactly = 1) {
            relativeAttitudeProcessorSpy.process(
                gyroscopeMeasurement,
                timestamp
            )
        }*/
        verify(relativeAttitudeProcessorSpy, times(1)).attitude
//        verify(exactly = 1) { relativeAttitudeProcessorSpy.attitude }
        verify(gravityProcessorSpy, only()).process(accelerometerMeasurement, timestamp)
//        verify(exactly = 1) { gravityProcessorSpy.process(accelerometerMeasurement, timestamp) }
    }

    @Test
    fun process_whenRelativeAttitudeProcessedPreviousRelativeGravityProcessedAndNoListener_updatesFusedAttitudeAndReturnsTrue() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor()

        // setup spies
        val relativeAttitudeProcessor: BaseRelativeGyroscopeAttitudeProcessor? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitudeProcessor"
        )
        requireNotNull(relativeAttitudeProcessor)
        val relativeAttitudeProcessorSpy = spy(relativeAttitudeProcessor)
//        val relativeAttitudeProcessorSpy = spyk(relativeAttitudeProcessor)
        doReturn(true).whenever(relativeAttitudeProcessorSpy).process(any(), any())
//        every { relativeAttitudeProcessorSpy.process(any(), any()) }.returns(true)
        val relativeAttitude = getAttitude()
        doReturn(relativeAttitude).whenever(relativeAttitudeProcessorSpy).attitude
//        every { relativeAttitudeProcessorSpy.attitude }.returns(relativeAttitude)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitudeProcessor",
            relativeAttitudeProcessorSpy
        )

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        doReturn(true).whenever(gravityProcessorSpy).process(any(), any())
//        every { gravityProcessorSpy.process(any(), any()) }.returns(true)
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextDouble()
        val gy = randomizer.nextDouble()
        val gz = randomizer.nextDouble()
        doReturn(gx).whenever(gravityProcessorSpy).gx
//        every { gravityProcessorSpy.gx }.returns(gx)
        doReturn(gy).whenever(gravityProcessorSpy).gy
//        every { gravityProcessorSpy.gy }.returns(gy)
        doReturn(gz).whenever(gravityProcessorSpy).gz
//        every { gravityProcessorSpy.gz }.returns(gz)
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val levelingProcessor: BaseLevelingProcessor? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "levelingProcessor"
        )
        requireNotNull(levelingProcessor)
        val levelingProcessorSpy = spy(levelingProcessor)
//        val levelingProcessorSpy = spyk(levelingProcessor)
        val levelingAttitude = getAttitude()
        doReturn(levelingAttitude).whenever(levelingProcessorSpy).attitude
//        every { levelingProcessorSpy.attitude }.returns(levelingAttitude)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "levelingProcessor",
            levelingProcessorSpy
        )

        val previousRelativeAttitude1 = getAttitude()
        val previousRelativeAttitudeCopy = Quaternion(previousRelativeAttitude1)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "previousRelativeAttitude",
            previousRelativeAttitude1
        )

        // check initial values
        val relativeAttitude1: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitude"
        )
        requireNotNull(relativeAttitude1)
        assertEquals(Quaternion(), relativeAttitude1)

        val timestamp1: Long? =
            getPrivateProperty(BaseLeveledRelativeAttitudeProcessor::class, processor, "timestamp")
        requireNotNull(timestamp1)
        assertEquals(0L, timestamp1)

        val panicCounter1: Int? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "panicCounter"
        )
        requireNotNull(panicCounter1)
        assertEquals(processor.panicCounterThreshold, panicCounter1)

        val resetToLeveling1: Boolean? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "resetToLeveling"
        )
        requireNotNull(resetToLeveling1)
        assertTrue(resetToLeveling1)

        // process
        val accelerometerMeasurement = AccelerometerSensorMeasurement()
        val gyroscopeMeasurement = GyroscopeSensorMeasurement()
        val timestamp = System.nanoTime()
        val syncedMeasurement = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gyroscopeMeasurement,
            timestamp
        )
        assertTrue(processor.process(syncedMeasurement))

        // check
        val relativeAttitude2: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitude"
        )
        requireNotNull(relativeAttitude2)
        assertEquals(relativeAttitude, relativeAttitude2)

        val previousRelativeAttitude2: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "previousRelativeAttitude"
        )
        requireNotNull(previousRelativeAttitude2)
        assertEquals(relativeAttitude, previousRelativeAttitude2)

        val inversePreviousRelativeAttitude: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "inversePreviousRelativeAttitude"
        )
        requireNotNull(inversePreviousRelativeAttitude)
        assertEquals(
            previousRelativeAttitudeCopy.inverseAndReturnNew(),
            inversePreviousRelativeAttitude
        )

        val deltaRelativeAttitude: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "deltaRelativeAttitude"
        )
        requireNotNull(deltaRelativeAttitude)
        assertEquals(
            relativeAttitude2.multiplyAndReturnNew(inversePreviousRelativeAttitude),
            deltaRelativeAttitude
        )

        val timestamp2: Long? =
            getPrivateProperty(BaseLeveledRelativeAttitudeProcessor::class, processor, "timestamp")
        requireNotNull(timestamp2)
        assertEquals(timestamp, timestamp2)

        val levelingAttitude1: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "levelingAttitude"
        )
        requireNotNull(levelingAttitude1)

        val eulerAngles1 = levelingAttitude.toEulerAngles()
        val levelingRoll = eulerAngles1[0]
        val levelingPitch = eulerAngles1[1]

        val eulerAngles2 = relativeAttitude.toEulerAngles()
        val yaw = eulerAngles2[2]
        val levelingAttitude2 = Quaternion(levelingRoll, levelingPitch, yaw)
        assertEquals(levelingAttitude2, levelingAttitude1)

        assertEquals(levelingAttitude2, processor.fusedAttitude)

        val panicCounter2: Int? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "panicCounter"
        )
        requireNotNull(panicCounter2)
        assertEquals(0, panicCounter2)

        val resetToLeveling2: Boolean? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "resetToLeveling"
        )
        requireNotNull(resetToLeveling2)
        assertFalse(resetToLeveling2)

        verify(relativeAttitudeProcessorSpy, times(1)).process(
            gyroscopeMeasurement,
            timestamp
        )
/*        verify(exactly = 1) {
            relativeAttitudeProcessorSpy.process(
                gyroscopeMeasurement,
                timestamp
            )
        }*/
        verify(relativeAttitudeProcessorSpy, times(1)).attitude
//        verify(exactly = 1) { relativeAttitudeProcessorSpy.attitude }
        verify(gravityProcessorSpy, times(1)).process(accelerometerMeasurement, timestamp)
//        verify(exactly = 1) { gravityProcessorSpy.process(accelerometerMeasurement, timestamp) }
        verify(gravityProcessorSpy, times(1)).gx
//        verify(exactly = 1) { gravityProcessorSpy.gx }
        verify(gravityProcessorSpy, times(1)).gy
//        verify(exactly = 1) { gravityProcessorSpy.gy }
        verify(gravityProcessorSpy, times(1)).gz
//        verify(exactly = 1) { gravityProcessorSpy.gz }
        verify(levelingProcessorSpy, times(1)).process(gx, gy, gz)
//        verify(exactly = 1) { levelingProcessorSpy.process(gx, gy, gz) }
        verify(levelingProcessorSpy, times(2)).attitude
//        verify(exactly = 2) { levelingProcessorSpy.attitude }
    }

    @Test
    fun process_whenRelativeAttitudeProcessedPreviousRelativeGravityProcessedAndListener_updatesFusedAttitudeAndReturnsTrue() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor(processorListener)

        // setup spies
        val relativeAttitudeProcessor: BaseRelativeGyroscopeAttitudeProcessor? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitudeProcessor"
        )
        requireNotNull(relativeAttitudeProcessor)
        val relativeAttitudeProcessorSpy = spy(relativeAttitudeProcessor)
//        val relativeAttitudeProcessorSpy = spyk(relativeAttitudeProcessor)
        doReturn(true).whenever(relativeAttitudeProcessorSpy).process(any(), any())
//        every { relativeAttitudeProcessorSpy.process(any(), any()) }.returns(true)
        val relativeAttitude = getAttitude()
        doReturn(relativeAttitude).whenever(relativeAttitudeProcessorSpy).attitude
//        every { relativeAttitudeProcessorSpy.attitude }.returns(relativeAttitude)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitudeProcessor",
            relativeAttitudeProcessorSpy
        )

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        doReturn(true).whenever(gravityProcessorSpy).process(any(), any())
//        every { gravityProcessorSpy.process(any(), any()) }.returns(true)
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextDouble()
        val gy = randomizer.nextDouble()
        val gz = randomizer.nextDouble()
        doReturn(gx).whenever(gravityProcessorSpy).gx
//        every { gravityProcessorSpy.gx }.returns(gx)
        doReturn(gy).whenever(gravityProcessorSpy).gy
//        every { gravityProcessorSpy.gy }.returns(gy)
        doReturn(gz).whenever(gravityProcessorSpy).gz
//        every { gravityProcessorSpy.gz }.returns(gz)
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val levelingProcessor: BaseLevelingProcessor? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "levelingProcessor"
        )
        requireNotNull(levelingProcessor)
        val levelingProcessorSpy = spy(levelingProcessor)
//        val levelingProcessorSpy = spyk(levelingProcessor)
        val levelingAttitude = getAttitude()
        doReturn(levelingAttitude).whenever(levelingProcessorSpy).attitude
//        every { levelingProcessorSpy.attitude }.returns(levelingAttitude)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "levelingProcessor",
            levelingProcessorSpy
        )

        val previousRelativeAttitude1 = getAttitude()
        val previousRelativeAttitudeCopy = Quaternion(previousRelativeAttitude1)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "previousRelativeAttitude",
            previousRelativeAttitude1
        )

        // check initial values
        val relativeAttitude1: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitude"
        )
        requireNotNull(relativeAttitude1)
        assertEquals(Quaternion(), relativeAttitude1)

        val timestamp1: Long? =
            getPrivateProperty(BaseLeveledRelativeAttitudeProcessor::class, processor, "timestamp")
        requireNotNull(timestamp1)
        assertEquals(0L, timestamp1)

        val panicCounter1: Int? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "panicCounter"
        )
        requireNotNull(panicCounter1)
        assertEquals(processor.panicCounterThreshold, panicCounter1)

        val resetToLeveling1: Boolean? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "resetToLeveling"
        )
        requireNotNull(resetToLeveling1)
        assertTrue(resetToLeveling1)

        // process
        val accelerometerMeasurement =
            AccelerometerSensorMeasurement(accuracy = SensorAccuracy.MEDIUM)
        val gyroscopeMeasurement = GyroscopeSensorMeasurement(accuracy = SensorAccuracy.HIGH)
        val timestamp = System.nanoTime()
        val syncedMeasurement = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gyroscopeMeasurement,
            timestamp
        )
        assertTrue(processor.process(syncedMeasurement))

        // check
        val relativeAttitude2: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitude"
        )
        requireNotNull(relativeAttitude2)
        assertEquals(relativeAttitude, relativeAttitude2)

        val previousRelativeAttitude2: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "previousRelativeAttitude"
        )
        requireNotNull(previousRelativeAttitude2)
        assertEquals(relativeAttitude, previousRelativeAttitude2)

        val inversePreviousRelativeAttitude: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "inversePreviousRelativeAttitude"
        )
        requireNotNull(inversePreviousRelativeAttitude)
        assertEquals(
            previousRelativeAttitudeCopy.inverseAndReturnNew(),
            inversePreviousRelativeAttitude
        )

        val deltaRelativeAttitude: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "deltaRelativeAttitude"
        )
        requireNotNull(deltaRelativeAttitude)
        assertEquals(
            relativeAttitude2.multiplyAndReturnNew(inversePreviousRelativeAttitude),
            deltaRelativeAttitude
        )

        val timestamp2: Long? =
            getPrivateProperty(BaseLeveledRelativeAttitudeProcessor::class, processor, "timestamp")
        requireNotNull(timestamp2)
        assertEquals(timestamp, timestamp2)

        val levelingAttitude1: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "levelingAttitude"
        )
        requireNotNull(levelingAttitude1)

        val eulerAngles1 = levelingAttitude.toEulerAngles()
        val levelingRoll = eulerAngles1[0]
        val levelingPitch = eulerAngles1[1]

        val eulerAngles2 = relativeAttitude.toEulerAngles()
        val yaw = eulerAngles2[2]
        val levelingAttitude2 = Quaternion(levelingRoll, levelingPitch, yaw)
        assertEquals(levelingAttitude2, levelingAttitude1)

        assertEquals(levelingAttitude2, processor.fusedAttitude)

        val panicCounter2: Int? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "panicCounter"
        )
        requireNotNull(panicCounter2)
        assertEquals(0, panicCounter2)

        val resetToLeveling2: Boolean? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "resetToLeveling"
        )
        requireNotNull(resetToLeveling2)
        assertFalse(resetToLeveling2)

        verify(relativeAttitudeProcessorSpy, times(1)).process(
            gyroscopeMeasurement,
            timestamp
        )
/*        verify(exactly = 1) {
            relativeAttitudeProcessorSpy.process(
                gyroscopeMeasurement,
                timestamp
            )
        }*/
        verify(relativeAttitudeProcessorSpy, times(1)).attitude
//        verify(exactly = 1) { relativeAttitudeProcessorSpy.attitude }
        verify(gravityProcessorSpy, times(1)).process(accelerometerMeasurement, timestamp)
//        verify(exactly = 1) { gravityProcessorSpy.process(accelerometerMeasurement, timestamp) }
        verify(gravityProcessorSpy, times(1)).gx
//        verify(exactly = 1) { gravityProcessorSpy.gx }
        verify(gravityProcessorSpy, times(1)).gy
//        verify(exactly = 1) { gravityProcessorSpy.gy }
        verify(gravityProcessorSpy, times(1)).gz
//        verify(exactly = 1) { gravityProcessorSpy.gz }
        verify(levelingProcessorSpy, times(1)).process(gx, gy, gz)
//        verify(exactly = 1) { levelingProcessorSpy.process(gx, gy, gz) }
        verify(levelingProcessorSpy, times(2)).attitude
//        verify(exactly = 2) { levelingProcessorSpy.attitude }

        verify(processorListener, only()).onProcessed(
            processor,
            processor.fusedAttitude,
            SensorAccuracy.MEDIUM,
            SensorAccuracy.HIGH
        )
/*        verify(exactly = 1) {
            processorListener.onProcessed(
                processor,
                processor.fusedAttitude,
                SensorAccuracy.MEDIUM,
                SensorAccuracy.HIGH
            )
        }*/
    }

    @Test
    fun process_whenRelativeAttitudeProcessedPreviousRelativeGravityProcessedAndNoResetToLevelingAndMediumDivergence_fusesAttitudeIncreasesPanicCounterAndReturnsTrue() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor(processorListener)

        // setup spies
        val relativeAttitudeProcessor: BaseRelativeGyroscopeAttitudeProcessor? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitudeProcessor"
        )
        requireNotNull(relativeAttitudeProcessor)
        val relativeAttitudeProcessorSpy = spy(relativeAttitudeProcessor)
//        val relativeAttitudeProcessorSpy = spyk(relativeAttitudeProcessor)
        doReturn(true).whenever(relativeAttitudeProcessorSpy).process(any(), any())
//        every { relativeAttitudeProcessorSpy.process(any(), any()) }.returns(true)
        val relativeAttitude = getAttitude()
        doReturn(relativeAttitude).whenever(relativeAttitudeProcessorSpy).attitude
//        every { relativeAttitudeProcessorSpy.attitude }.returns(relativeAttitude)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitudeProcessor",
            relativeAttitudeProcessorSpy
        )

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        doReturn(true).whenever(gravityProcessorSpy).process(any(), any())
//        every { gravityProcessorSpy.process(any(), any()) }.returns(true)
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextDouble()
        val gy = randomizer.nextDouble()
        val gz = randomizer.nextDouble()
        doReturn(gx).whenever(gravityProcessorSpy).gx
//        every { gravityProcessorSpy.gx }.returns(gx)
        doReturn(gy).whenever(gravityProcessorSpy).gy
//        every { gravityProcessorSpy.gy }.returns(gy)
        doReturn(gz).whenever(gravityProcessorSpy).gz
//        every { gravityProcessorSpy.gz }.returns(gz)
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val levelingProcessor: BaseLevelingProcessor? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "levelingProcessor"
        )
        requireNotNull(levelingProcessor)
        val levelingProcessorSpy = spy(levelingProcessor)
//        val levelingProcessorSpy = spyk(levelingProcessor)
        val levelingAttitude = getAttitude()
        doReturn(levelingAttitude).whenever(levelingProcessorSpy).attitude
//        every { levelingProcessorSpy.attitude }.returns(levelingAttitude)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "levelingProcessor",
            levelingProcessorSpy
        )

        val previousRelativeAttitude1 = getAttitude()
        val previousRelativeAttitudeCopy = Quaternion(previousRelativeAttitude1)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "previousRelativeAttitude",
            previousRelativeAttitude1
        )

        // check initial values
        val relativeAttitude1: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitude"
        )
        requireNotNull(relativeAttitude1)
        assertEquals(Quaternion(), relativeAttitude1)

        val timestamp1: Long? =
            getPrivateProperty(BaseLeveledRelativeAttitudeProcessor::class, processor, "timestamp")
        requireNotNull(timestamp1)
        assertEquals(0L, timestamp1)

        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "panicCounter",
            0
        )

        val resetToLeveling1: Boolean? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "resetToLeveling"
        )
        requireNotNull(resetToLeveling1)
        assertFalse(resetToLeveling1)

        val fusedAttitude1 = getAttitude()
        val fusedAttitudeCopy = Quaternion(fusedAttitude1)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "fusedAttitude",
            fusedAttitude1
        )

        mockkObject(QuaternionHelper) {
            every {
                QuaternionHelper.dotProduct(
                    any(),
                    any()
                )
            }.returns(processor.outlierPanicThreshold)

            // process
            val accelerometerMeasurement =
                AccelerometerSensorMeasurement(accuracy = SensorAccuracy.MEDIUM)
            val gyroscopeMeasurement = GyroscopeSensorMeasurement(accuracy = SensorAccuracy.HIGH)
            val timestamp = System.nanoTime()
            val syncedMeasurement = AccelerometerAndGyroscopeSyncedSensorMeasurement(
                accelerometerMeasurement,
                gyroscopeMeasurement,
                timestamp
            )
            assertTrue(processor.process(syncedMeasurement))

            // check
            val relativeAttitude2: Quaternion? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "relativeAttitude"
            )
            requireNotNull(relativeAttitude2)
            assertEquals(relativeAttitude, relativeAttitude2)

            val previousRelativeAttitude2: Quaternion? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "previousRelativeAttitude"
            )
            requireNotNull(previousRelativeAttitude2)
            assertEquals(relativeAttitude, previousRelativeAttitude2)

            val inversePreviousRelativeAttitude: Quaternion? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "inversePreviousRelativeAttitude"
            )
            requireNotNull(inversePreviousRelativeAttitude)
            assertEquals(
                previousRelativeAttitudeCopy.inverseAndReturnNew(),
                inversePreviousRelativeAttitude
            )

            val deltaRelativeAttitude: Quaternion? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "deltaRelativeAttitude"
            )
            requireNotNull(deltaRelativeAttitude)
            assertEquals(
                relativeAttitude2.multiplyAndReturnNew(inversePreviousRelativeAttitude),
                deltaRelativeAttitude
            )

            val timestamp2: Long? =
                getPrivateProperty(
                    BaseLeveledRelativeAttitudeProcessor::class,
                    processor,
                    "timestamp"
                )
            requireNotNull(timestamp2)
            assertEquals(timestamp, timestamp2)

            val levelingAttitude1: Quaternion? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "levelingAttitude"
            )
            requireNotNull(levelingAttitude1)

            val eulerAngles1 = levelingAttitude.toEulerAngles()
            val levelingRoll = eulerAngles1[0]
            val levelingPitch = eulerAngles1[1]

            val eulerAngles2 = relativeAttitude.toEulerAngles()
            val yaw = eulerAngles2[2]
            val levelingAttitude2 = Quaternion(levelingRoll, levelingPitch, yaw)
            assertEquals(levelingAttitude2, levelingAttitude1)

            val fusedAttitude2 = deltaRelativeAttitude.multiplyAndReturnNew(fusedAttitudeCopy)
            assertEquals(fusedAttitude2, processor.fusedAttitude)

            val panicCounter2: Int? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "panicCounter"
            )
            requireNotNull(panicCounter2)
            assertEquals(0, panicCounter2)

            val resetToLeveling2: Boolean? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "resetToLeveling"
            )
            requireNotNull(resetToLeveling2)
            assertFalse(resetToLeveling2)

            verify(relativeAttitudeProcessorSpy, times(1)).process(
                gyroscopeMeasurement,
                timestamp
            )
/*            verify(exactly = 1) {
                relativeAttitudeProcessorSpy.process(
                    gyroscopeMeasurement,
                    timestamp
                )
            }*/
            verify(relativeAttitudeProcessorSpy, times(1)).attitude
//            verify(exactly = 1) { relativeAttitudeProcessorSpy.attitude }
            verify(gravityProcessorSpy, times(1)).process(accelerometerMeasurement, timestamp)
//            verify(exactly = 1) { gravityProcessorSpy.process(accelerometerMeasurement, timestamp) }
            verify(gravityProcessorSpy, times(1)).gx
//            verify(exactly = 1) { gravityProcessorSpy.gx }
            verify(gravityProcessorSpy, times(1)).gy
//            verify(exactly = 1) { gravityProcessorSpy.gy }
            verify(gravityProcessorSpy, times(1)).gz
//            verify(exactly = 1) { gravityProcessorSpy.gz }
            verify(levelingProcessorSpy, times(1)).process(gx, gy, gz)
//            verify(exactly = 1) { levelingProcessorSpy.process(gx, gy, gz) }
            verify(levelingProcessorSpy, times(2)).attitude
//            verify(exactly = 2) { levelingProcessorSpy.attitude }

            verify(processorListener, only()).onProcessed(
                processor,
                processor.fusedAttitude,
                SensorAccuracy.MEDIUM,
                SensorAccuracy.HIGH
            )
/*            verify(exactly = 1) {
                processorListener.onProcessed(
                    processor,
                    processor.fusedAttitude,
                    SensorAccuracy.MEDIUM,
                    SensorAccuracy.HIGH
                )
            }*/
        }
    }

    @Test
    fun process_whenRelativeAttitudeProcessedPreviousRelativeGravityProcessedAndNoResetToLevelingAndLargeDivergence_fusesAttitudeIncreasesPanicCounterAndReturnsTrue() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor(processorListener)

        // setup spies
        val relativeAttitudeProcessor: BaseRelativeGyroscopeAttitudeProcessor? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitudeProcessor"
        )
        requireNotNull(relativeAttitudeProcessor)
        val relativeAttitudeProcessorSpy = spy(relativeAttitudeProcessor)
//        val relativeAttitudeProcessorSpy = spyk(relativeAttitudeProcessor)
        doReturn(true).whenever(relativeAttitudeProcessorSpy).process(any(), any())
//        every { relativeAttitudeProcessorSpy.process(any(), any()) }.returns(true)
        val relativeAttitude = getAttitude()
        doReturn(relativeAttitude).whenever(relativeAttitudeProcessorSpy).attitude
//        every { relativeAttitudeProcessorSpy.attitude }.returns(relativeAttitude)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitudeProcessor",
            relativeAttitudeProcessorSpy
        )

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        doReturn(true).whenever(gravityProcessorSpy).process(any(), any())
//        every { gravityProcessorSpy.process(any(), any()) }.returns(true)
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextDouble()
        val gy = randomizer.nextDouble()
        val gz = randomizer.nextDouble()
        doReturn(gx).whenever(gravityProcessorSpy).gx
//        every { gravityProcessorSpy.gx }.returns(gx)
        doReturn(gy).whenever(gravityProcessorSpy).gy
//        every { gravityProcessorSpy.gy }.returns(gy)
        doReturn(gz).whenever(gravityProcessorSpy).gz
//        every { gravityProcessorSpy.gz }.returns(gz)
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val levelingProcessor: BaseLevelingProcessor? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "levelingProcessor"
        )
        requireNotNull(levelingProcessor)
        val levelingProcessorSpy = spy(levelingProcessor)
//        val levelingProcessorSpy = spyk(levelingProcessor)
        val levelingAttitude = getAttitude()
        doReturn(levelingAttitude).whenever(levelingProcessorSpy).attitude
//        every { levelingProcessorSpy.attitude }.returns(levelingAttitude)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "levelingProcessor",
            levelingProcessorSpy
        )

        val previousRelativeAttitude1 = getAttitude()
        val previousRelativeAttitudeCopy = Quaternion(previousRelativeAttitude1)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "previousRelativeAttitude",
            previousRelativeAttitude1
        )

        // check initial values
        val relativeAttitude1: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitude"
        )
        requireNotNull(relativeAttitude1)
        assertEquals(Quaternion(), relativeAttitude1)

        val timestamp1: Long? =
            getPrivateProperty(BaseLeveledRelativeAttitudeProcessor::class, processor, "timestamp")
        requireNotNull(timestamp1)
        assertEquals(0L, timestamp1)

        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "panicCounter",
            0
        )

        val resetToLeveling1: Boolean? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "resetToLeveling"
        )
        requireNotNull(resetToLeveling1)
        assertFalse(resetToLeveling1)

        val fusedAttitude1 = getAttitude()
        val fusedAttitudeCopy = Quaternion(fusedAttitude1)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "fusedAttitude",
            fusedAttitude1
        )

        mockkObject(QuaternionHelper) {
            every { QuaternionHelper.dotProduct(any(), any()) }.returns(0.0)

            // process
            val accelerometerMeasurement =
                AccelerometerSensorMeasurement(accuracy = SensorAccuracy.MEDIUM)
            val gyroscopeMeasurement = GyroscopeSensorMeasurement(accuracy = SensorAccuracy.HIGH)
            val timestamp = System.nanoTime()
            val syncedMeasurement = AccelerometerAndGyroscopeSyncedSensorMeasurement(
                accelerometerMeasurement,
                gyroscopeMeasurement,
                timestamp
            )
            assertTrue(processor.process(syncedMeasurement))

            // check
            val relativeAttitude2: Quaternion? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "relativeAttitude"
            )
            requireNotNull(relativeAttitude2)
            assertEquals(relativeAttitude, relativeAttitude2)

            val previousRelativeAttitude2: Quaternion? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "previousRelativeAttitude"
            )
            requireNotNull(previousRelativeAttitude2)
            assertEquals(relativeAttitude, previousRelativeAttitude2)

            val inversePreviousRelativeAttitude: Quaternion? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "inversePreviousRelativeAttitude"
            )
            requireNotNull(inversePreviousRelativeAttitude)
            assertEquals(
                previousRelativeAttitudeCopy.inverseAndReturnNew(),
                inversePreviousRelativeAttitude
            )

            val deltaRelativeAttitude: Quaternion? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "deltaRelativeAttitude"
            )
            requireNotNull(deltaRelativeAttitude)
            assertEquals(
                relativeAttitude2.multiplyAndReturnNew(inversePreviousRelativeAttitude),
                deltaRelativeAttitude
            )

            val timestamp2: Long? =
                getPrivateProperty(
                    BaseLeveledRelativeAttitudeProcessor::class,
                    processor,
                    "timestamp"
                )
            requireNotNull(timestamp2)
            assertEquals(timestamp, timestamp2)

            val levelingAttitude1: Quaternion? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "levelingAttitude"
            )
            requireNotNull(levelingAttitude1)

            val eulerAngles1 = levelingAttitude.toEulerAngles()
            val levelingRoll = eulerAngles1[0]
            val levelingPitch = eulerAngles1[1]

            val eulerAngles2 = relativeAttitude.toEulerAngles()
            val yaw = eulerAngles2[2]
            val levelingAttitude2 = Quaternion(levelingRoll, levelingPitch, yaw)
            assertEquals(levelingAttitude2, levelingAttitude1)

            val fusedAttitude2 = deltaRelativeAttitude.multiplyAndReturnNew(fusedAttitudeCopy)
            assertEquals(fusedAttitude2, processor.fusedAttitude)

            val panicCounter2: Int? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "panicCounter"
            )
            requireNotNull(panicCounter2)
            assertEquals(1, panicCounter2)

            val resetToLeveling2: Boolean? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "resetToLeveling"
            )
            requireNotNull(resetToLeveling2)
            assertFalse(resetToLeveling2)

            verify(relativeAttitudeProcessorSpy, times(1)).process(
                gyroscopeMeasurement,
                timestamp
            )
/*            verify(exactly = 1) {
                relativeAttitudeProcessorSpy.process(
                    gyroscopeMeasurement,
                    timestamp
                )
            }*/
            verify(relativeAttitudeProcessorSpy, times(1)).attitude
//            verify(exactly = 1) { relativeAttitudeProcessorSpy.attitude }
            verify(gravityProcessorSpy, times(1)).process(accelerometerMeasurement, timestamp)
//            verify(exactly = 1) { gravityProcessorSpy.process(accelerometerMeasurement, timestamp) }
            verify(gravityProcessorSpy, times(1)).gx
//            verify(exactly = 1) { gravityProcessorSpy.gx }
            verify(gravityProcessorSpy, times(1)).gy
//            verify(exactly = 1) { gravityProcessorSpy.gy }
            verify(gravityProcessorSpy, times(1)).gz
//            verify(exactly = 1) { gravityProcessorSpy.gz }
            verify(levelingProcessorSpy, times(1)).process(gx, gy, gz)
//            verify(exactly = 1) { levelingProcessorSpy.process(gx, gy, gz) }
            verify(levelingProcessorSpy, times(2)).attitude
//            verify(exactly = 2) { levelingProcessorSpy.attitude }

            verify(processorListener, only()).onProcessed(
                processor,
                processor.fusedAttitude,
                SensorAccuracy.MEDIUM,
                SensorAccuracy.HIGH
            )
/*            verify(exactly = 1) {
                processorListener.onProcessed(
                    processor,
                    processor.fusedAttitude,
                    SensorAccuracy.MEDIUM,
                    SensorAccuracy.HIGH
                )
            }*/
        }
    }

    @Test
    fun process_whenRelativeAttitudeProcessedPreviousRelativeGravityProcessedAndNoResetToLevelingSmallDivergenceAndIndirectInterpolation_fusesAttitudeIncreasesPanicCounterAndReturnsTrue() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor(processorListener)
        processor.useIndirectInterpolation = true

        // setup spies
        val relativeAttitudeProcessor: BaseRelativeGyroscopeAttitudeProcessor? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitudeProcessor"
        )
        requireNotNull(relativeAttitudeProcessor)
        val relativeAttitudeProcessorSpy = spy(relativeAttitudeProcessor)
//        val relativeAttitudeProcessorSpy = spyk(relativeAttitudeProcessor)
        doReturn(true).whenever(relativeAttitudeProcessorSpy).process(any(), any())
//        every { relativeAttitudeProcessorSpy.process(any(), any()) }.returns(true)
        val relativeAttitude = getAttitude()
        doReturn(relativeAttitude).whenever(relativeAttitudeProcessorSpy).attitude
//        every { relativeAttitudeProcessorSpy.attitude }.returns(relativeAttitude)
        doReturn(TIME_INTERVAL).whenever(relativeAttitudeProcessorSpy).timeIntervalSeconds
//        every { relativeAttitudeProcessorSpy.timeIntervalSeconds }.returns(TIME_INTERVAL)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitudeProcessor",
            relativeAttitudeProcessorSpy
        )

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        doReturn(true).whenever(gravityProcessorSpy).process(any(), any())
//        every { gravityProcessorSpy.process(any(), any()) }.returns(true)
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextDouble()
        val gy = randomizer.nextDouble()
        val gz = randomizer.nextDouble()
        doReturn(gx).whenever(gravityProcessorSpy).gx
//        every { gravityProcessorSpy.gx }.returns(gx)
        doReturn(gy).whenever(gravityProcessorSpy).gy
//        every { gravityProcessorSpy.gy }.returns(gy)
        doReturn(gz).whenever(gravityProcessorSpy).gz
//        every { gravityProcessorSpy.gz }.returns(gz)
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val levelingProcessor: BaseLevelingProcessor? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "levelingProcessor"
        )
        requireNotNull(levelingProcessor)
        val levelingProcessorSpy = spy(levelingProcessor)
//        val levelingProcessorSpy = spyk(levelingProcessor)
        val levelingAttitude = getAttitude()
        doReturn(levelingAttitude).whenever(levelingProcessorSpy).attitude
//        every { levelingProcessorSpy.attitude }.returns(levelingAttitude)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "levelingProcessor",
            levelingProcessorSpy
        )

        val previousRelativeAttitude1 = getAttitude()
        val previousRelativeAttitudeCopy = Quaternion(previousRelativeAttitude1)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "previousRelativeAttitude",
            previousRelativeAttitude1
        )

        // check initial values
        val relativeAttitude1: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitude"
        )
        requireNotNull(relativeAttitude1)
        assertEquals(Quaternion(), relativeAttitude1)

        val timestamp1: Long? =
            getPrivateProperty(BaseLeveledRelativeAttitudeProcessor::class, processor, "timestamp")
        requireNotNull(timestamp1)
        assertEquals(0L, timestamp1)

        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "panicCounter",
            0
        )

        val resetToLeveling1: Boolean? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "resetToLeveling"
        )
        requireNotNull(resetToLeveling1)
        assertFalse(resetToLeveling1)

        val fusedAttitude1 = getAttitude()
        val fusedAttitudeCopy = Quaternion(fusedAttitude1)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "fusedAttitude",
            fusedAttitude1
        )

        mockkObject(QuaternionHelper) {
            every { QuaternionHelper.dotProduct(any(), any()) }.returns(processor.outlierThreshold)

            // process
            val accelerometerMeasurement =
                AccelerometerSensorMeasurement(accuracy = SensorAccuracy.MEDIUM)
            val gyroscopeMeasurement = GyroscopeSensorMeasurement(accuracy = SensorAccuracy.HIGH)
            val timestamp = System.nanoTime()
            val syncedMeasurement = AccelerometerAndGyroscopeSyncedSensorMeasurement(
                accelerometerMeasurement,
                gyroscopeMeasurement,
                timestamp
            )
            assertTrue(processor.process(syncedMeasurement))

            // check
            val relativeAttitude2: Quaternion? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "relativeAttitude"
            )
            requireNotNull(relativeAttitude2)
            assertEquals(relativeAttitude, relativeAttitude2)

            val previousRelativeAttitude2: Quaternion? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "previousRelativeAttitude"
            )
            requireNotNull(previousRelativeAttitude2)
            assertEquals(relativeAttitude, previousRelativeAttitude2)

            val inversePreviousRelativeAttitude: Quaternion? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "inversePreviousRelativeAttitude"
            )
            requireNotNull(inversePreviousRelativeAttitude)
            assertEquals(
                previousRelativeAttitudeCopy.inverseAndReturnNew(),
                inversePreviousRelativeAttitude
            )

            val deltaRelativeAttitude: Quaternion? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "deltaRelativeAttitude"
            )
            requireNotNull(deltaRelativeAttitude)
            assertEquals(
                relativeAttitude2.multiplyAndReturnNew(inversePreviousRelativeAttitude),
                deltaRelativeAttitude
            )

            val timestamp2: Long? =
                getPrivateProperty(
                    BaseLeveledRelativeAttitudeProcessor::class,
                    processor,
                    "timestamp"
                )
            requireNotNull(timestamp2)
            assertEquals(timestamp, timestamp2)

            val levelingAttitude1: Quaternion? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "levelingAttitude"
            )
            requireNotNull(levelingAttitude1)

            val eulerAngles1 = levelingAttitude.toEulerAngles()
            val levelingRoll = eulerAngles1[0]
            val levelingPitch = eulerAngles1[1]

            val eulerAngles2 = relativeAttitude.toEulerAngles()
            val yaw = eulerAngles2[2]
            val levelingAttitude2 = Quaternion(levelingRoll, levelingPitch, yaw)
            assertEquals(levelingAttitude2, levelingAttitude1)

            val fusedAttitude2 = deltaRelativeAttitude.multiplyAndReturnNew(fusedAttitudeCopy)
            val t = min(
                processor.interpolationValue + processor.indirectInterpolationWeight *
                        abs(deltaRelativeAttitude.rotationAngle / TIME_INTERVAL), 1.0
            )
            Quaternion.slerp(
                fusedAttitude2,
                levelingAttitude2,
                t,
                fusedAttitude2
            )
            assertEquals(fusedAttitude2, processor.fusedAttitude)

            val panicCounter2: Int? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "panicCounter"
            )
            requireNotNull(panicCounter2)
            assertEquals(0, panicCounter2)

            val resetToLeveling2: Boolean? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "resetToLeveling"
            )
            requireNotNull(resetToLeveling2)
            assertFalse(resetToLeveling2)

            verify(relativeAttitudeProcessorSpy, times(1)).process(
                gyroscopeMeasurement,
                timestamp
            )
/*            verify(exactly = 1) {
                relativeAttitudeProcessorSpy.process(
                    gyroscopeMeasurement,
                    timestamp
                )
            }*/
            verify(relativeAttitudeProcessorSpy, times(1)).attitude
//            verify(exactly = 1) { relativeAttitudeProcessorSpy.attitude }
            verify(gravityProcessorSpy, times(1)).process(accelerometerMeasurement, timestamp)
//            verify(exactly = 1) { gravityProcessorSpy.process(accelerometerMeasurement, timestamp) }
            verify(gravityProcessorSpy, times(1)).gx
//            verify(exactly = 1) { gravityProcessorSpy.gx }
            verify(gravityProcessorSpy, times(1)).gy
//            verify(exactly = 1) { gravityProcessorSpy.gy }
            verify(gravityProcessorSpy, times(1)).gz
//            verify(exactly = 1) { gravityProcessorSpy.gz }
            verify(levelingProcessorSpy, times(1)).process(gx, gy, gz)
//            verify(exactly = 1) { levelingProcessorSpy.process(gx, gy, gz) }
            verify(levelingProcessorSpy, times(2)).attitude
//            verify(exactly = 2) { levelingProcessorSpy.attitude }

            verify(processorListener, only()).onProcessed(
                processor,
                processor.fusedAttitude,
                SensorAccuracy.MEDIUM,
                SensorAccuracy.HIGH
            )
/*            verify(exactly = 1) {
                processorListener.onProcessed(
                    processor,
                    processor.fusedAttitude,
                    SensorAccuracy.MEDIUM,
                    SensorAccuracy.HIGH
                )
            }*/
        }
    }

    @Test
    fun process_whenRelativeAttitudeProcessedPreviousRelativeGravityProcessedAndNoResetToLevelingSmallDivergenceAndDirectInterpolation_fusesAttitudeIncreasesPanicCounterAndReturnsTrue() {
        val processor = AccelerometerLeveledRelativeAttitudeProcessor(processorListener)
        processor.useIndirectInterpolation = false

        // setup spies
        val relativeAttitudeProcessor: BaseRelativeGyroscopeAttitudeProcessor? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitudeProcessor"
        )
        requireNotNull(relativeAttitudeProcessor)
        val relativeAttitudeProcessorSpy = spy(relativeAttitudeProcessor)
//        val relativeAttitudeProcessorSpy = spyk(relativeAttitudeProcessor)
        doReturn(true).whenever(relativeAttitudeProcessorSpy).process(any(), any())
//        every { relativeAttitudeProcessorSpy.process(any(), any()) }.returns(true)
        val relativeAttitude = getAttitude()
        doReturn(relativeAttitude).whenever(relativeAttitudeProcessorSpy).attitude
//        every { relativeAttitudeProcessorSpy.attitude }.returns(relativeAttitude)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitudeProcessor",
            relativeAttitudeProcessorSpy
        )

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        doReturn(true).whenever(gravityProcessorSpy).process(any(), any())
//        every { gravityProcessorSpy.process(any(), any()) }.returns(true)
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextDouble()
        val gy = randomizer.nextDouble()
        val gz = randomizer.nextDouble()
        doReturn(gx).whenever(gravityProcessorSpy).gx
//        every { gravityProcessorSpy.gx }.returns(gx)
        doReturn(gy).whenever(gravityProcessorSpy).gy
//        every { gravityProcessorSpy.gy }.returns(gy)
        doReturn(gz).whenever(gravityProcessorSpy).gz
//        every { gravityProcessorSpy.gz }.returns(gz)
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val levelingProcessor: BaseLevelingProcessor? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "levelingProcessor"
        )
        requireNotNull(levelingProcessor)
        val levelingProcessorSpy = spy(levelingProcessor)
//        val levelingProcessorSpy = spyk(levelingProcessor)
        val levelingAttitude = getAttitude()
        doReturn(levelingAttitude).whenever(levelingProcessorSpy).attitude
//        every { levelingProcessorSpy.attitude }.returns(levelingAttitude)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "levelingProcessor",
            levelingProcessorSpy
        )

        val previousRelativeAttitude1 = getAttitude()
        val previousRelativeAttitudeCopy = Quaternion(previousRelativeAttitude1)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "previousRelativeAttitude",
            previousRelativeAttitude1
        )

        // check initial values
        val relativeAttitude1: Quaternion? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "relativeAttitude"
        )
        requireNotNull(relativeAttitude1)
        assertEquals(Quaternion(), relativeAttitude1)

        val timestamp1: Long? =
            getPrivateProperty(BaseLeveledRelativeAttitudeProcessor::class, processor, "timestamp")
        requireNotNull(timestamp1)
        assertEquals(0L, timestamp1)

        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "panicCounter",
            0
        )

        val resetToLeveling1: Boolean? = getPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "resetToLeveling"
        )
        requireNotNull(resetToLeveling1)
        assertFalse(resetToLeveling1)

        val fusedAttitude1 = getAttitude()
        val fusedAttitudeCopy = Quaternion(fusedAttitude1)
        setPrivateProperty(
            BaseLeveledRelativeAttitudeProcessor::class,
            processor,
            "fusedAttitude",
            fusedAttitude1
        )

        mockkObject(QuaternionHelper) {
            every { QuaternionHelper.dotProduct(any(), any()) }.returns(processor.outlierThreshold)

            // process
            val accelerometerMeasurement =
                AccelerometerSensorMeasurement(accuracy = SensorAccuracy.MEDIUM)
            val gyroscopeMeasurement = GyroscopeSensorMeasurement(accuracy = SensorAccuracy.HIGH)
            val timestamp = System.nanoTime()
            val syncedMeasurement = AccelerometerAndGyroscopeSyncedSensorMeasurement(
                accelerometerMeasurement,
                gyroscopeMeasurement,
                timestamp
            )
            assertTrue(processor.process(syncedMeasurement))

            // check
            val relativeAttitude2: Quaternion? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "relativeAttitude"
            )
            requireNotNull(relativeAttitude2)
            assertEquals(relativeAttitude, relativeAttitude2)

            val previousRelativeAttitude2: Quaternion? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "previousRelativeAttitude"
            )
            requireNotNull(previousRelativeAttitude2)
            assertEquals(relativeAttitude, previousRelativeAttitude2)

            val inversePreviousRelativeAttitude: Quaternion? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "inversePreviousRelativeAttitude"
            )
            requireNotNull(inversePreviousRelativeAttitude)
            assertEquals(
                previousRelativeAttitudeCopy.inverseAndReturnNew(),
                inversePreviousRelativeAttitude
            )

            val deltaRelativeAttitude: Quaternion? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "deltaRelativeAttitude"
            )
            requireNotNull(deltaRelativeAttitude)
            assertEquals(
                relativeAttitude2.multiplyAndReturnNew(inversePreviousRelativeAttitude),
                deltaRelativeAttitude
            )

            val timestamp2: Long? =
                getPrivateProperty(
                    BaseLeveledRelativeAttitudeProcessor::class,
                    processor,
                    "timestamp"
                )
            requireNotNull(timestamp2)
            assertEquals(timestamp, timestamp2)

            val levelingAttitude1: Quaternion? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "levelingAttitude"
            )
            requireNotNull(levelingAttitude1)

            val eulerAngles1 = levelingAttitude.toEulerAngles()
            val levelingRoll = eulerAngles1[0]
            val levelingPitch = eulerAngles1[1]

            val eulerAngles2 = relativeAttitude.toEulerAngles()
            val yaw = eulerAngles2[2]
            val levelingAttitude2 = Quaternion(levelingRoll, levelingPitch, yaw)
            assertEquals(levelingAttitude2, levelingAttitude1)

            val fusedAttitude2 = deltaRelativeAttitude.multiplyAndReturnNew(fusedAttitudeCopy)
            Quaternion.slerp(
                fusedAttitude2,
                levelingAttitude2,
                processor.interpolationValue,
                fusedAttitude2
            )
            assertEquals(fusedAttitude2, processor.fusedAttitude)

            val panicCounter2: Int? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "panicCounter"
            )
            requireNotNull(panicCounter2)
            assertEquals(0, panicCounter2)

            val resetToLeveling2: Boolean? = getPrivateProperty(
                BaseLeveledRelativeAttitudeProcessor::class,
                processor,
                "resetToLeveling"
            )
            requireNotNull(resetToLeveling2)
            assertFalse(resetToLeveling2)

            verify(relativeAttitudeProcessorSpy, times(1)).process(
                gyroscopeMeasurement,
                timestamp
            )
/*            verify(exactly = 1) {
                relativeAttitudeProcessorSpy.process(
                    gyroscopeMeasurement,
                    timestamp
                )
            }*/
            verify(relativeAttitudeProcessorSpy, times(1)).attitude
//            verify(exactly = 1) { relativeAttitudeProcessorSpy.attitude }
            verify(gravityProcessorSpy, times(1)).process(accelerometerMeasurement, timestamp)
//            verify(exactly = 1) { gravityProcessorSpy.process(accelerometerMeasurement, timestamp) }
            verify(gravityProcessorSpy, times(1)).gx
//            verify(exactly = 1) { gravityProcessorSpy.gx }
            verify(gravityProcessorSpy, times(1)).gy
//            verify(exactly = 1) { gravityProcessorSpy.gy }
            verify(gravityProcessorSpy, times(1)).gz
//            verify(exactly = 1) { gravityProcessorSpy.gz }
            verify(levelingProcessorSpy, times(1)).process(gx, gy, gz)
//            verify(exactly = 1) { levelingProcessorSpy.process(gx, gy, gz) }
            verify(levelingProcessorSpy, times(2)).attitude
//            verify(exactly = 2) { levelingProcessorSpy.attitude }

            verify(processorListener, only()).onProcessed(
                processor,
                processor.fusedAttitude,
                SensorAccuracy.MEDIUM,
                SensorAccuracy.HIGH
            )
/*            verify(exactly = 1) {
                processorListener.onProcessed(
                    processor,
                    processor.fusedAttitude,
                    SensorAccuracy.MEDIUM,
                    SensorAccuracy.HIGH
                )
            }*/
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

        whenever(location.latitude).thenReturn(latitudeDegrees)
//        every { location.latitude }.returns(latitudeDegrees)
        whenever(location.longitude).thenReturn(longitudeDegrees)
//        every { location.longitude }.returns(longitudeDegrees)
        whenever(location.altitude).thenReturn(height)
//        every { location.altitude }.returns(height)

        return location
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

        fun getAttitude(): Quaternion {
            val randomizer = UniformRandomizer()
            val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))

            return Quaternion(roll, pitch, yaw)
        }
    }
}