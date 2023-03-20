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
import com.irurueta.android.navigation.inertial.collectors.AccelerometerAndMagnetometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.navigation.inertial.estimators.AttitudeEstimator
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.AccelerationUnit
import com.irurueta.units.MagneticFluxDensityConverter
import com.irurueta.units.MagneticFluxDensityUnit
import io.mockk.*
import org.junit.After
import org.junit.Assert.*
import org.junit.Test
import java.util.*

class AccelerometerGeomagneticAttitudeProcessorTest {

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun constructor_whenNoParameters_returnsExpectedValues() {
        val processor = AccelerometerGeomagneticAttitudeProcessor()

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
    }

    @Test
    fun constructor_whenListener_returnsExpectedValues() {
        val listener =
            mockk<BaseGeomagneticAttitudeProcessor.OnProcessedListener<AccelerometerSensorMeasurement,
                    AccelerometerAndMagnetometerSyncedSensorMeasurement>>()
        val processor = AccelerometerGeomagneticAttitudeProcessor(listener)

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
    }

    @Test
    fun processorListener_setsExpectedValue() {
        val processor = AccelerometerGeomagneticAttitudeProcessor()

        // check default value
        assertNull(processor.processorListener)

        val listener =
            mockk<BaseGeomagneticAttitudeProcessor.OnProcessedListener<AccelerometerSensorMeasurement,
                    AccelerometerAndMagnetometerSyncedSensorMeasurement>>()
        processor.processorListener = listener

        // check
        assertSame(listener, processor.processorListener)
    }

    @Test
    fun gx_returnsExpectedValue() {
        val processor = AccelerometerGeomagneticAttitudeProcessor()

        // setup spy
        val gravityProcessor: AccelerometerGravityProcessor? = processor.getPrivateProperty(
            "gravityProcessor"
        )
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextDouble()
        every { gravityProcessorSpy.gx }.returns(gx)
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        assertEquals(gx, processor.gx, 0.0)
        verify(exactly = 1) { gravityProcessorSpy.gx }
    }

    @Test
    fun gy_returnsExpectedValue() {
        val processor = AccelerometerGeomagneticAttitudeProcessor()

        // setup spy
        val gravityProcessor: AccelerometerGravityProcessor? = processor.getPrivateProperty(
            "gravityProcessor"
        )
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        val randomizer = UniformRandomizer()
        val gy = randomizer.nextDouble()
        val gz = randomizer.nextDouble()
        every { gravityProcessorSpy.gy }.returns(gy)
        every { gravityProcessorSpy.gz }.returns(gz)
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        assertEquals(gy, processor.gy, 0.0)
        verify(exactly = 1) { gravityProcessorSpy.gy }
    }

    @Test
    fun gz_returnsExpectedValue() {
        val processor = AccelerometerGeomagneticAttitudeProcessor()

        // setup spy
        val gravityProcessor: AccelerometerGravityProcessor? = processor.getPrivateProperty(
            "gravityProcessor"
        )
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        val randomizer = UniformRandomizer()
        val gz = randomizer.nextDouble()
        every { gravityProcessorSpy.gz }.returns(gz)
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        assertEquals(gz, processor.gz, 0.0)
        verify(exactly = 1) { gravityProcessorSpy.gz }
    }

    @Test
    fun gravity_returnsExpectedValue() {
        val processor = AccelerometerGeomagneticAttitudeProcessor()

        // setup spy
        val gravityProcessor: AccelerometerGravityProcessor? = processor.getPrivateProperty(
            "gravityProcessor"
        )
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        val gravity = AccelerationTriad()
        every { gravityProcessorSpy.gravity }.returns(gravity)
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        assertSame(gravity, processor.gravity)
        verify(exactly = 1) { gravityProcessorSpy.gravity }
    }

    @Test
    fun getGravity_returnsExpectedValue() {
        val processor = AccelerometerGeomagneticAttitudeProcessor()

        // setup spy
        val gravityProcessor: AccelerometerGravityProcessor? = processor.getPrivateProperty(
            "gravityProcessor"
        )
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextDouble()
        val gy = randomizer.nextDouble()
        val gz = randomizer.nextDouble()
        every { gravityProcessorSpy.getGravity(any()) }.answers { answer ->
            val triad = answer.invocation.args[0] as AccelerationTriad
            triad.setValueCoordinatesAndUnit(gx, gy, gz, AccelerationUnit.METERS_PER_SQUARED_SECOND)
        }
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val gravity = AccelerationTriad()
        processor.getGravity(gravity)

        // check
        assertEquals(gx, gravity.valueX, 0.0)
        assertEquals(gy, gravity.valueY, 0.0)
        assertEquals(gz, gravity.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, gravity.unit)
    }

    @Test
    fun location_setsExpectedValue() {
        val processor = AccelerometerGeomagneticAttitudeProcessor()

        // check default value
        assertNull(processor.location)

        // set nw value
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
        val processor = AccelerometerGeomagneticAttitudeProcessor()

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

    @Test
    fun currentDate_setsExpectedValue() {
        val processor = AccelerometerGeomagneticAttitudeProcessor()

        // check default value
        assertNull(processor.currentDate)

        // set new value
        val currentDate = Date()
        processor.currentDate = currentDate

        // check
        assertSame(currentDate, processor.currentDate)
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateLevelingProcessor_whenTrueAndLocationUnavailable_throwsIllegalStateException() {
        val processor = AccelerometerGeomagneticAttitudeProcessor()

        // check default value
        assertNull(processor.location)
        assertFalse(processor.useAccurateLevelingProcessor)

        // set new value
        processor.useAccurateLevelingProcessor = true
    }

    @Test
    fun useAccurateLevelingProcessor_whenValid_setsExpectedValue() {
        val processor = AccelerometerGeomagneticAttitudeProcessor()

        // check default value
        assertNull(processor.location)
        assertFalse(processor.useAccurateLevelingProcessor)

        val levelingProcessor1: BaseLevelingProcessor? =
            getPrivateProperty(
                BaseGeomagneticAttitudeProcessor::class,
                processor,
                "levelingProcessor"
            )
        requireNotNull(levelingProcessor1)
        assertTrue(levelingProcessor1 is LevelingProcessor)

        // set location
        val location = getLocation()
        processor.location = location

        // set to true
        processor.useAccurateLevelingProcessor = true

        // check
        assertSame(location, processor.location)
        assertTrue(processor.useAccurateLevelingProcessor)

        val levelingProcessor2: BaseLevelingProcessor? =
            getPrivateProperty(
                BaseGeomagneticAttitudeProcessor::class,
                processor,
                "levelingProcessor"
            )
        requireNotNull(levelingProcessor2)
        assertTrue(levelingProcessor2 is AccurateLevelingProcessor)

        // set to false
        processor.useAccurateLevelingProcessor = false

        // check
        assertFalse(processor.useAccurateLevelingProcessor)

        val levelingProcessor3: BaseLevelingProcessor? =
            getPrivateProperty(
                BaseGeomagneticAttitudeProcessor::class,
                processor,
                "levelingProcessor"
            )
        requireNotNull(levelingProcessor3)
        assertTrue(levelingProcessor3 is LevelingProcessor)
    }

    @Test
    fun worldMagneticModel_whenWorldMagneticModelNotUsed_setsExpectedValue() {
        val processor = AccelerometerGeomagneticAttitudeProcessor()

        // check default value
        assertNull(processor.worldMagneticModel)
        assertFalse(processor.useWorldMagneticModel)

        val wmmEstimator1: WMMEarthMagneticFluxDensityEstimator? =
            getPrivateProperty(BaseGeomagneticAttitudeProcessor::class, processor, "wmmEstimator")
        assertNull(wmmEstimator1)

        // set new value
        val model = WorldMagneticModel()
        processor.worldMagneticModel = model

        // check
        assertSame(model, processor.worldMagneticModel)

        val wmmEstimator2: WMMEarthMagneticFluxDensityEstimator? =
            getPrivateProperty(BaseGeomagneticAttitudeProcessor::class, processor, "wmmEstimator")
        assertNull(wmmEstimator2)
    }

    @Test
    fun worldMagneticModel_whenWorldMagneticModelUsed_setsExpectedValue() {
        val processor = AccelerometerGeomagneticAttitudeProcessor()

        // check default value
        assertNull(processor.worldMagneticModel)
        assertFalse(processor.useWorldMagneticModel)

        val wmmEstimator1: WMMEarthMagneticFluxDensityEstimator? =
            getPrivateProperty(BaseGeomagneticAttitudeProcessor::class, processor, "wmmEstimator")
        assertNull(wmmEstimator1)

        // enable
        processor.useWorldMagneticModel = true

        // check
        val wmmEstimator2: WMMEarthMagneticFluxDensityEstimator? =
            getPrivateProperty(BaseGeomagneticAttitudeProcessor::class, processor, "wmmEstimator")
        requireNotNull(wmmEstimator2)
        assertNotNull(wmmEstimator2.model)

        // set model
        val model = WorldMagneticModel()
        processor.worldMagneticModel = model

        // check
        val wmmEstimator3: WMMEarthMagneticFluxDensityEstimator? =
            getPrivateProperty(BaseGeomagneticAttitudeProcessor::class, processor, "wmmEstimator")
        requireNotNull(wmmEstimator3)
        assertSame(model, wmmEstimator3.model)
    }

    @Test
    fun useWorldMagneticModel_setsExpectedValue() {
        val processor = AccelerometerGeomagneticAttitudeProcessor()

        // check default value
        assertFalse(processor.useWorldMagneticModel)

        val wmmEstimator1: WMMEarthMagneticFluxDensityEstimator? =
            getPrivateProperty(BaseGeomagneticAttitudeProcessor::class, processor, "wmmEstimator")
        assertNull(wmmEstimator1)

        // enable
        processor.useWorldMagneticModel = true

        // check
        assertTrue(processor.useWorldMagneticModel)

        val wmmEstimator2: WMMEarthMagneticFluxDensityEstimator? =
            getPrivateProperty(BaseGeomagneticAttitudeProcessor::class, processor, "wmmEstimator")
        requireNotNull(wmmEstimator2)
        assertNotNull(wmmEstimator2.model)
    }

    @Test
    fun process_whenEmpty_returnsFalse() {
        val processor = AccelerometerGeomagneticAttitudeProcessor()

        val syncedMeasurement = AccelerometerAndMagnetometerSyncedSensorMeasurement()
        assertFalse(processor.process(syncedMeasurement))
    }

    @Test
    fun process_whenNoMagnetometerMeasurement_returnsFalse() {
        val processor = AccelerometerGeomagneticAttitudeProcessor()

        val syncedMeasurement = AccelerometerAndMagnetometerSyncedSensorMeasurement(
            AccelerometerSensorMeasurement()
        )
        assertFalse(processor.process(syncedMeasurement))
    }

    @Test
    fun process_whenGravityNotProcessed_returnsFalse() {
        val processor = AccelerometerGeomagneticAttitudeProcessor()

        val gravityProcessor: AccelerometerGravityProcessor? = processor.getPrivateProperty(
            "gravityProcessor"
        )
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(any()) }.returns(false)
        processor.setPrivateProperty(
            "gravityProcessor",
            gravityProcessorSpy
        )

        val accelerometerMeasurement = AccelerometerSensorMeasurement()
        val syncedMeasurement = AccelerometerAndMagnetometerSyncedSensorMeasurement(
            accelerometerMeasurement,
            MagnetometerSensorMeasurement()
        )
        assertFalse(processor.process(syncedMeasurement))

        verify(exactly = 1) { gravityProcessorSpy.process(accelerometerMeasurement) }
    }

    @Test
    fun process_whenGravityProcessedAndHardIronDefined_returnsTrue() {
        val processor = AccelerometerGeomagneticAttitudeProcessor()

        val gravityProcessor: AccelerometerGravityProcessor? = processor.getPrivateProperty(
            "gravityProcessor"
        )
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(any()) }.returns(true)
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextDouble()
        val gy = randomizer.nextDouble()
        val gz = randomizer.nextDouble()
        every { gravityProcessorSpy.gx }.returns(gx)
        every { gravityProcessorSpy.gy }.returns(gy)
        every { gravityProcessorSpy.gz }.returns(gz)
        processor.setPrivateProperty(
            "gravityProcessor",
            gravityProcessorSpy
        )

        val levelingProcessor: BaseLevelingProcessor? = getPrivateProperty(
            BaseGeomagneticAttitudeProcessor::class,
            processor,
            "levelingProcessor"
        )
        requireNotNull(levelingProcessor)
        val levelingProcessorSpy = spyk(levelingProcessor)
        val levelingAttitude1 = getAttitude()
        every { levelingProcessorSpy.attitude }.returns(levelingAttitude1)
        setPrivateProperty(
            BaseGeomagneticAttitudeProcessor::class,
            processor,
            "levelingProcessor",
            levelingProcessorSpy
        )

        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val magnetometerMeasurement =
            MagnetometerSensorMeasurement(bx, by, bz, hardIronX, hardIronY, hardIronZ)
        val syncedMeasurement = AccelerometerAndMagnetometerSyncedSensorMeasurement(
            AccelerometerSensorMeasurement(),
            magnetometerMeasurement
        )
        assertTrue(processor.process(syncedMeasurement))

        every { levelingProcessorSpy.process(gx, gy, gz) }

        val triad: MagneticFluxDensityTriad? =
            getPrivateProperty(BaseGeomagneticAttitudeProcessor::class, processor, "triad")
        requireNotNull(triad)
        assertEquals(
            MagneticFluxDensityConverter.microTeslaToTesla((bx - hardIronX).toDouble()),
            triad.valueY,
            0.0
        )
        assertEquals(
            MagneticFluxDensityConverter.microTeslaToTesla((by - hardIronY).toDouble()),
            triad.valueX,
            0.0
        )
        assertEquals(
            MagneticFluxDensityConverter.microTeslaToTesla((bz - hardIronZ).toDouble()),
            -triad.valueZ,
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, triad.unit)

        val levelingAttitude2: Quaternion? = getPrivateProperty(
            BaseGeomagneticAttitudeProcessor::class,
            processor,
            "levelingAttitude"
        )
        requireNotNull(levelingAttitude2)
        assertEquals(levelingAttitude1, levelingAttitude2)

        val eulerAngles1 = levelingAttitude1.toEulerAngles()

        val eulerAngles2: DoubleArray? =
            getPrivateProperty(BaseGeomagneticAttitudeProcessor::class, processor, "eulerAngles")
        requireNotNull(eulerAngles2)
        assertArrayEquals(eulerAngles1, eulerAngles2, 0.0)

        val roll1 = eulerAngles1[0]
        val pitch1 = eulerAngles1[1]
        val declination = 0.0
        val yaw1 = AttitudeEstimator.getYaw(
            triad.valueX,
            triad.valueY,
            triad.valueZ,
            declination,
            roll1,
            pitch1
        )

        val fusedAttitude = Quaternion(roll1, pitch1, yaw1)
        assertEquals(fusedAttitude, processor.fusedAttitude)
    }

    @Test
    fun process_whenGravityProcessedAndHardIronNotDefined_returnsTrue() {
        val processor = AccelerometerGeomagneticAttitudeProcessor()

        val gravityProcessor: AccelerometerGravityProcessor? = processor.getPrivateProperty(
            "gravityProcessor"
        )
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(any()) }.returns(true)
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextDouble()
        val gy = randomizer.nextDouble()
        val gz = randomizer.nextDouble()
        every { gravityProcessorSpy.gx }.returns(gx)
        every { gravityProcessorSpy.gy }.returns(gy)
        every { gravityProcessorSpy.gz }.returns(gz)
        processor.setPrivateProperty(
            "gravityProcessor",
            gravityProcessorSpy
        )

        val levelingProcessor: BaseLevelingProcessor? = getPrivateProperty(
            BaseGeomagneticAttitudeProcessor::class,
            processor,
            "levelingProcessor"
        )
        requireNotNull(levelingProcessor)
        val levelingProcessorSpy = spyk(levelingProcessor)
        val levelingAttitude1 = getAttitude()
        every { levelingProcessorSpy.attitude }.returns(levelingAttitude1)
        setPrivateProperty(
            BaseGeomagneticAttitudeProcessor::class,
            processor,
            "levelingProcessor",
            levelingProcessorSpy
        )

        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val magnetometerMeasurement =
            MagnetometerSensorMeasurement(bx, by, bz)
        val syncedMeasurement = AccelerometerAndMagnetometerSyncedSensorMeasurement(
            AccelerometerSensorMeasurement(),
            magnetometerMeasurement
        )
        assertTrue(processor.process(syncedMeasurement))

        every { levelingProcessorSpy.process(gx, gy, gz) }

        val triad: MagneticFluxDensityTriad? =
            getPrivateProperty(BaseGeomagneticAttitudeProcessor::class, processor, "triad")
        requireNotNull(triad)
        assertEquals(
            MagneticFluxDensityConverter.microTeslaToTesla(bx.toDouble()),
            triad.valueY,
            0.0
        )
        assertEquals(
            MagneticFluxDensityConverter.microTeslaToTesla(by.toDouble()),
            triad.valueX,
            0.0
        )
        assertEquals(
            MagneticFluxDensityConverter.microTeslaToTesla(bz.toDouble()),
            -triad.valueZ,
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, triad.unit)

        val levelingAttitude2: Quaternion? = getPrivateProperty(
            BaseGeomagneticAttitudeProcessor::class,
            processor,
            "levelingAttitude"
        )
        requireNotNull(levelingAttitude2)
        assertEquals(levelingAttitude1, levelingAttitude2)

        val eulerAngles1 = levelingAttitude1.toEulerAngles()

        val eulerAngles2: DoubleArray? =
            getPrivateProperty(BaseGeomagneticAttitudeProcessor::class, processor, "eulerAngles")
        requireNotNull(eulerAngles2)
        assertArrayEquals(eulerAngles1, eulerAngles2, 0.0)

        val roll1 = eulerAngles1[0]
        val pitch1 = eulerAngles1[1]
        val declination = 0.0
        val yaw1 = AttitudeEstimator.getYaw(
            triad.valueX,
            triad.valueY,
            triad.valueZ,
            declination,
            roll1,
            pitch1
        )

        val fusedAttitude = Quaternion(roll1, pitch1, yaw1)
        assertEquals(fusedAttitude, processor.fusedAttitude)
    }

    @Test
    fun process_whenWmmModelUsedAndLocation_returnsTrue() {
        val processor = AccelerometerGeomagneticAttitudeProcessor()
        val location = getLocation()
        processor.location = location
        processor.useWorldMagneticModel = true

        val gravityProcessor: AccelerometerGravityProcessor? = processor.getPrivateProperty(
            "gravityProcessor"
        )
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(any()) }.returns(true)
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextDouble()
        val gy = randomizer.nextDouble()
        val gz = randomizer.nextDouble()
        every { gravityProcessorSpy.gx }.returns(gx)
        every { gravityProcessorSpy.gy }.returns(gy)
        every { gravityProcessorSpy.gz }.returns(gz)
        processor.setPrivateProperty(
            "gravityProcessor",
            gravityProcessorSpy
        )

        val levelingProcessor: BaseLevelingProcessor? = getPrivateProperty(
            BaseGeomagneticAttitudeProcessor::class,
            processor,
            "levelingProcessor"
        )
        requireNotNull(levelingProcessor)
        val levelingProcessorSpy = spyk(levelingProcessor)
        val levelingAttitude1 = getAttitude()
        every { levelingProcessorSpy.attitude }.returns(levelingAttitude1)
        setPrivateProperty(
            BaseGeomagneticAttitudeProcessor::class,
            processor,
            "levelingProcessor",
            levelingProcessorSpy
        )

        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val magnetometerMeasurement =
            MagnetometerSensorMeasurement(bx, by, bz)
        val syncedMeasurement = AccelerometerAndMagnetometerSyncedSensorMeasurement(
            AccelerometerSensorMeasurement(),
            magnetometerMeasurement
        )
        assertTrue(processor.process(syncedMeasurement))

        every { levelingProcessorSpy.process(gx, gy, gz) }

        val triad: MagneticFluxDensityTriad? =
            getPrivateProperty(BaseGeomagneticAttitudeProcessor::class, processor, "triad")
        requireNotNull(triad)
        assertEquals(
            MagneticFluxDensityConverter.microTeslaToTesla(bx.toDouble()),
            triad.valueY,
            0.0
        )
        assertEquals(
            MagneticFluxDensityConverter.microTeslaToTesla(by.toDouble()),
            triad.valueX,
            0.0
        )
        assertEquals(
            MagneticFluxDensityConverter.microTeslaToTesla(bz.toDouble()),
            -triad.valueZ,
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, triad.unit)

        val levelingAttitude2: Quaternion? = getPrivateProperty(
            BaseGeomagneticAttitudeProcessor::class,
            processor,
            "levelingAttitude"
        )
        requireNotNull(levelingAttitude2)
        assertEquals(levelingAttitude1, levelingAttitude2)

        val eulerAngles1 = levelingAttitude1.toEulerAngles()

        val eulerAngles2: DoubleArray? =
            getPrivateProperty(BaseGeomagneticAttitudeProcessor::class, processor, "eulerAngles")
        requireNotNull(eulerAngles2)
        assertArrayEquals(eulerAngles1, eulerAngles2, 0.0)

        val roll1 = eulerAngles1[0]
        val pitch1 = eulerAngles1[1]
        val wmmEstimator = WMMEarthMagneticFluxDensityEstimator()
        val position = location.toNEDPosition()
        val declination = wmmEstimator.getDeclination(
            position.latitude,
            position.longitude,
            position.height,
            Date()
        )
        val yaw1 = AttitudeEstimator.getYaw(
            triad.valueX,
            triad.valueY,
            triad.valueZ,
            declination,
            roll1,
            pitch1
        )

        val fusedAttitude = Quaternion(roll1, pitch1, yaw1)
        assertEquals(fusedAttitude, processor.fusedAttitude)
    }

    @Test
    fun process_whenProcessorListener_returnsTrueAndNotifies() {
        val listener =
            mockk<BaseGeomagneticAttitudeProcessor.OnProcessedListener<AccelerometerSensorMeasurement,
                    AccelerometerAndMagnetometerSyncedSensorMeasurement>>(relaxUnitFun = true)
        val processor = AccelerometerGeomagneticAttitudeProcessor(listener)
        val location = getLocation()
        processor.location = location
        processor.useWorldMagneticModel = true

        val gravityProcessor: AccelerometerGravityProcessor? = processor.getPrivateProperty(
            "gravityProcessor"
        )
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(any()) }.returns(true)
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextDouble()
        val gy = randomizer.nextDouble()
        val gz = randomizer.nextDouble()
        every { gravityProcessorSpy.gx }.returns(gx)
        every { gravityProcessorSpy.gy }.returns(gy)
        every { gravityProcessorSpy.gz }.returns(gz)
        processor.setPrivateProperty(
            "gravityProcessor",
            gravityProcessorSpy
        )

        val levelingProcessor: BaseLevelingProcessor? = getPrivateProperty(
            BaseGeomagneticAttitudeProcessor::class,
            processor,
            "levelingProcessor"
        )
        requireNotNull(levelingProcessor)
        val levelingProcessorSpy = spyk(levelingProcessor)
        val levelingAttitude1 = getAttitude()
        every { levelingProcessorSpy.attitude }.returns(levelingAttitude1)
        setPrivateProperty(
            BaseGeomagneticAttitudeProcessor::class,
            processor,
            "levelingProcessor",
            levelingProcessorSpy
        )

        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val magnetometerMeasurement =
            MagnetometerSensorMeasurement(bx, by, bz, accuracy = SensorAccuracy.MEDIUM)
        val syncedMeasurement = AccelerometerAndMagnetometerSyncedSensorMeasurement(
            AccelerometerSensorMeasurement(accuracy = SensorAccuracy.HIGH),
            magnetometerMeasurement
        )
        assertTrue(processor.process(syncedMeasurement))

        every { levelingProcessorSpy.process(gx, gy, gz) }

        val triad: MagneticFluxDensityTriad? =
            getPrivateProperty(BaseGeomagneticAttitudeProcessor::class, processor, "triad")
        requireNotNull(triad)
        assertEquals(
            MagneticFluxDensityConverter.microTeslaToTesla(bx.toDouble()),
            triad.valueY,
            0.0
        )
        assertEquals(
            MagneticFluxDensityConverter.microTeslaToTesla(by.toDouble()),
            triad.valueX,
            0.0
        )
        assertEquals(
            MagneticFluxDensityConverter.microTeslaToTesla(bz.toDouble()),
            -triad.valueZ,
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.TESLA, triad.unit)

        val levelingAttitude2: Quaternion? = getPrivateProperty(
            BaseGeomagneticAttitudeProcessor::class,
            processor,
            "levelingAttitude"
        )
        requireNotNull(levelingAttitude2)
        assertEquals(levelingAttitude1, levelingAttitude2)

        val eulerAngles1 = levelingAttitude1.toEulerAngles()

        val eulerAngles2: DoubleArray? =
            getPrivateProperty(BaseGeomagneticAttitudeProcessor::class, processor, "eulerAngles")
        requireNotNull(eulerAngles2)
        assertArrayEquals(eulerAngles1, eulerAngles2, 0.0)

        val roll1 = eulerAngles1[0]
        val pitch1 = eulerAngles1[1]
        val wmmEstimator = WMMEarthMagneticFluxDensityEstimator()
        val position = location.toNEDPosition()
        val declination = wmmEstimator.getDeclination(
            position.latitude,
            position.longitude,
            position.height,
            Date()
        )
        val yaw1 = AttitudeEstimator.getYaw(
            triad.valueX,
            triad.valueY,
            triad.valueZ,
            declination,
            roll1,
            pitch1
        )

        val fusedAttitude = Quaternion(roll1, pitch1, yaw1)
        assertEquals(fusedAttitude, processor.fusedAttitude)

        verify(exactly = 1) {
            listener.onProcessed(
                processor,
                processor.fusedAttitude,
                SensorAccuracy.HIGH,
                SensorAccuracy.MEDIUM
            )
        }
    }

    @Test
    fun reset_resetsInternalProcessorsAndFusedAttitude() {
        val processor = AccelerometerGeomagneticAttitudeProcessor()
        val fusedAttitude = getAttitude()
        fusedAttitude.copyTo(processor.fusedAttitude)

        val gravityProcessor: AccelerometerGravityProcessor? = processor.getPrivateProperty(
            "gravityProcessor"
        )
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val levelingProcessor: BaseLevelingProcessor? = getPrivateProperty(
            BaseGeomagneticAttitudeProcessor::class,
            processor,
            "levelingProcessor"
        )
        requireNotNull(levelingProcessor)
        val levelingProcessorSpy = spyk(levelingProcessor)
        setPrivateProperty(
            BaseGeomagneticAttitudeProcessor::class,
            processor,
            "levelingProcessor",
            levelingProcessorSpy
        )

        // reset
        processor.reset()

        // check
        verify(exactly = 1) { gravityProcessorSpy.reset() }
        verify(exactly = 1) { levelingProcessorSpy.reset() }
        assertEquals(Quaternion(), processor.fusedAttitude)
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