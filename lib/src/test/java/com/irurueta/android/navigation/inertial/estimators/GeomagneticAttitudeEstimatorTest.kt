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
package com.irurueta.android.navigation.inertial.estimators

import android.content.Context
import android.location.Location
import android.os.SystemClock
import android.view.Display
import android.view.Surface
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.navigation.inertial.callPrivateFunc
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MedianAveragingFilter
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.MagneticFluxDensityConverter
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import java.util.*

@RunWith(RobolectricTestRunner::class)
class GeomagneticAttitudeEstimatorTest {

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator(context)

        // check
        assertSame(context, estimator.context)
        assertNull(estimator.location)
        assertEquals(SensorDelay.GAME, estimator.sensorDelay)
        assertFalse(estimator.useAccelerometer)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
            estimator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER,
            estimator.magnetometerSensorType
        )
        assertNotNull(estimator.accelerometerAveragingFilter)
        assertTrue(estimator.accelerometerAveragingFilter is LowPassAveragingFilter)
        assertNull(estimator.worldMagneticModel)
        assertNotNull(estimator.timestamp)
        assertFalse(estimator.useWorldMagneticModel)
        assertFalse(estimator.useAccurateLevelingEstimator)
        assertFalse(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateDisplayEulerAngles)
        assertFalse(estimator.ignoreDisplayOrientation)
        assertNull(estimator.attitudeAvailableListener)
        assertFalse(estimator.running)
    }

    @Test
    fun constructor_whenAllProperties_setsEpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val filter = MedianAveragingFilter()
        val worldMagneticModel = WorldMagneticModel()
        val timestamp = Date()
        val listener = mockk<GeomagneticAttitudeEstimator.OnAttitudeAvailableListener>()
        val estimator = GeomagneticAttitudeEstimator(
            context,
            location,
            SensorDelay.NORMAL,
            useAccelerometer = true,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            filter,
            worldMagneticModel,
            timestamp,
            useWorldMagneticModel = true,
            useAccurateLevelingEstimator = true,
            estimateCoordinateTransformation = true,
            estimateDisplayEulerAngles = false,
            ignoreDisplayOrientation = true,
            listener
        )

        // check
        assertSame(context, estimator.context)
        assertSame(location, estimator.location)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertTrue(estimator.useAccelerometer)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            estimator.magnetometerSensorType
        )
        assertSame(filter, estimator.accelerometerAveragingFilter)
        assertSame(worldMagneticModel, estimator.worldMagneticModel)
        assertSame(timestamp, estimator.timestamp)
        assertTrue(estimator.useWorldMagneticModel)
        assertTrue(estimator.useAccurateLevelingEstimator)
        assertTrue(estimator.estimateCoordinateTransformation)
        assertFalse(estimator.estimateDisplayEulerAngles)
        assertTrue(estimator.ignoreDisplayOrientation)
        assertSame(listener, estimator.attitudeAvailableListener)
        assertFalse(estimator.running)
    }

    @Test(expected = IllegalStateException::class)
    fun constructor_whenAccurateLevelingEnabledAndMissingLocation_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        GeomagneticAttitudeEstimator(context, useAccurateLevelingEstimator = true)
    }

    @Test
    fun timestamp_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator(context)

        // check default value
        assertNotNull(estimator.timestamp)

        val timestamp = Date()
        estimator.timestamp = timestamp

        // check
        assertSame(timestamp, estimator.timestamp)
    }

    @Test
    fun attitudeAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator(context)

        // check default value
        assertNull(estimator.attitudeAvailableListener)

        // set new value
        val listener = mockk<GeomagneticAttitudeEstimator.OnAttitudeAvailableListener>()
        estimator.attitudeAvailableListener = listener

        // check
        assertSame(listener, estimator.attitudeAvailableListener)
    }

    @Test
    fun location_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator(context)

        assertFalse(estimator.running)
        assertNull(estimator.location)

        // set new value
        val location = getLocation()
        estimator.location = location

        // check
        assertSame(location, estimator.location)

        // set new value
        estimator.location = null

        // check
        assertNull(estimator.location)
    }

    @Test
    fun location_whenRunningAndNotNull_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator(context)

        assertFalse(estimator.running)
        assertNull(estimator.location)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        val location = getLocation()
        estimator.location = location

        // check
        assertSame(location, estimator.location)
    }

    @Test(expected = IllegalStateException::class)
    fun location_whenRunningAndNull_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = GeomagneticAttitudeEstimator(context, location)

        assertFalse(estimator.running)
        assertSame(location, estimator.location)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        // set new value
        estimator.location = null
    }

    @Test
    fun useAccurateLevelingEstimator_whenTrueValueNotRunningAndLocationAvailable_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = GeomagneticAttitudeEstimator(context, location)

        // check default value
        assertFalse(estimator.useAccurateLevelingEstimator)
        assertFalse(estimator.running)
        assertSame(location, estimator.location)

        // set new value
        estimator.useAccurateLevelingEstimator = true

        // check
        assertTrue(estimator.useAccurateLevelingEstimator)

        val levelingEstimator: AccurateLevelingEstimator? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)
        assertSame(context, levelingEstimator.context)
        assertSame(location, levelingEstimator.location)
        assertEquals(estimator.sensorDelay, levelingEstimator.sensorDelay)
        assertEquals(estimator.useAccelerometer, levelingEstimator.useAccelerometer)
        assertEquals(estimator.accelerometerSensorType, levelingEstimator.accelerometerSensorType)
        assertSame(
            estimator.accelerometerAveragingFilter,
            levelingEstimator.accelerometerAveragingFilter
        )
        assertFalse(levelingEstimator.estimateCoordinateTransformation)
        assertFalse(levelingEstimator.estimateDisplayEulerAngles)
        assertNotNull(levelingEstimator.levelingAvailableListener)
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateLevelingEstimator_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        estimator.useAccurateLevelingEstimator = true
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateLevelingEstimator_whenNotRunningAndNoLocation_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator(context)

        // check default values
        assertFalse(estimator.running)
        assertNull(estimator.location)
        assertFalse(estimator.useAccurateLevelingEstimator)

        // set new value
        estimator.useAccurateLevelingEstimator = true
    }

    @Test
    fun useAccurateLevelingEstimator_whenFalseValueNotRunningAndNoLocation_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator(context)

        // check default values
        assertFalse(estimator.running)
        assertNull(estimator.location)
        assertFalse(estimator.useAccurateLevelingEstimator)

        // set new value
        estimator.useAccurateLevelingEstimator = false

        // check
        assertFalse(estimator.useAccurateLevelingEstimator)

        val levelingEstimator: LevelingEstimator? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)
        assertSame(context, levelingEstimator.context)
        assertEquals(estimator.sensorDelay, levelingEstimator.sensorDelay)
        assertEquals(estimator.useAccelerometer, levelingEstimator.useAccelerometer)
        assertEquals(estimator.accelerometerSensorType, levelingEstimator.accelerometerSensorType)
        assertSame(
            estimator.accelerometerAveragingFilter,
            levelingEstimator.accelerometerAveragingFilter
        )
        assertFalse(levelingEstimator.estimateCoordinateTransformation)
        assertFalse(levelingEstimator.estimateDisplayEulerAngles)
        assertNotNull(levelingEstimator.levelingAvailableListener)
    }

    @Test
    fun worldMagneticModel_whenNotRunningAndNotUseWorldMagneticModel_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator(context)

        // check default values
        assertNull(estimator.worldMagneticModel)
        assertFalse(estimator.running)
        assertFalse(estimator.useWorldMagneticModel)

        // set new value
        val worldMagneticModel = WorldMagneticModel()
        estimator.worldMagneticModel = worldMagneticModel

        // check
        assertSame(worldMagneticModel, estimator.worldMagneticModel)

        val wmmEstimator1: WMMEarthMagneticFluxDensityEstimator? =
            estimator.getPrivateProperty("wmmEstimator")
        assertNull(wmmEstimator1)

        // set new value
        estimator.worldMagneticModel = null

        // check
        assertNull(estimator.worldMagneticModel)

        val wmmEstimator2: WMMEarthMagneticFluxDensityEstimator? =
            estimator.getPrivateProperty("wmmEstimator")
        assertNull(wmmEstimator2)
    }

    @Test
    fun worldMagneticModel_whenNotRunningAndUseWorldMagneticModel_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator(context, useWorldMagneticModel = true)

        // check default values
        assertNull(estimator.worldMagneticModel)
        assertFalse(estimator.running)
        assertTrue(estimator.useWorldMagneticModel)

        // set new value
        val worldMagneticModel = WorldMagneticModel()
        estimator.worldMagneticModel = worldMagneticModel

        // check
        assertSame(worldMagneticModel, estimator.worldMagneticModel)

        val wmmEstimator1: WMMEarthMagneticFluxDensityEstimator? =
            estimator.getPrivateProperty("wmmEstimator")
        requireNotNull(wmmEstimator1)
        assertSame(worldMagneticModel, wmmEstimator1.model)

        // set new value
        estimator.worldMagneticModel = null

        // check
        assertNull(estimator.worldMagneticModel)

        val wmmEstimator2: WMMEarthMagneticFluxDensityEstimator? =
            estimator.getPrivateProperty("wmmEstimator")
        requireNotNull(wmmEstimator2)
        assertNotSame(worldMagneticModel, wmmEstimator2.model)
    }

    @Test(expected = IllegalStateException::class)
    fun worldMagneticModel_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        // set new value
        val worldMagneticModel = WorldMagneticModel()
        estimator.worldMagneticModel = worldMagneticModel
    }

    @Test
    fun useWorldMagneticModel_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator(context)

        // check default values
        assertFalse(estimator.running)
        assertFalse(estimator.useWorldMagneticModel)

        // set new value
        estimator.useWorldMagneticModel = true

        // check
        assertTrue(estimator.useWorldMagneticModel)

        val wmmEstimator1: WMMEarthMagneticFluxDensityEstimator? =
            estimator.getPrivateProperty("wmmEstimator")
        requireNotNull(wmmEstimator1)
        assertNotNull(wmmEstimator1.model)

        // set new value
        estimator.useWorldMagneticModel = false

        // check
        assertFalse(estimator.useWorldMagneticModel)

        val wmmEstimator2: WMMEarthMagneticFluxDensityEstimator? =
            estimator.getPrivateProperty("wmmEstimator")
        assertNull(wmmEstimator2)
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        // start
        estimator.start()
    }

    @Test
    fun start_whenNotRunningAndLevelingEstimatorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator(context)

        // setup spies
        val levelingEstimator: BaseLevelingEstimator<*, *>? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)
        val levelingEstimatorSpy = spyk(levelingEstimator)
        every { levelingEstimatorSpy.start() }.returns(false)
        estimator.setPrivateProperty("levelingEstimator", levelingEstimatorSpy)

        val magnetometerSensorCollector: MagnetometerSensorCollector? =
            estimator.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        every { magnetometerSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        estimator.setPrivateProperty("hasMagnetometerValues", true)

        val hasMagnetometerValues1: Boolean? = estimator.getPrivateProperty("hasMagnetometerValues")
        requireNotNull(hasMagnetometerValues1)
        assertTrue(hasMagnetometerValues1)

        // check
        assertFalse(estimator.running)

        // start
        assertFalse(estimator.start())

        verify(exactly = 1) { levelingEstimatorSpy.start() }
        verify(exactly = 0) { magnetometerSensorCollectorSpy.start() }
        verify(exactly = 1) { levelingEstimatorSpy.stop() }
        verify(exactly = 1) { magnetometerSensorCollectorSpy.stop() }

        val hasMagnetometerValues2: Boolean? = estimator.getPrivateProperty("hasMagnetometerValues")
        requireNotNull(hasMagnetometerValues2)
        assertFalse(hasMagnetometerValues2)
    }

    @Test
    fun start_whenNotRunningAndMagnetometerSensorCollectorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator(context)

        // setup spies
        val levelingEstimator: BaseLevelingEstimator<*, *>? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)
        val levelingEstimatorSpy = spyk(levelingEstimator)
        every { levelingEstimatorSpy.start() }.returns(true)
        estimator.setPrivateProperty("levelingEstimator", levelingEstimatorSpy)

        val magnetometerSensorCollector: MagnetometerSensorCollector? =
            estimator.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        every { magnetometerSensorCollectorSpy.start() }.returns(false)
        estimator.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        estimator.setPrivateProperty("hasMagnetometerValues", true)

        val hasMagnetometerValues1: Boolean? = estimator.getPrivateProperty("hasMagnetometerValues")
        requireNotNull(hasMagnetometerValues1)
        assertTrue(hasMagnetometerValues1)

        // check
        assertFalse(estimator.running)

        // start
        assertFalse(estimator.start())

        verify(exactly = 1) { levelingEstimatorSpy.start() }
        verify(exactly = 1) { magnetometerSensorCollectorSpy.start() }
        verify(exactly = 1) { levelingEstimatorSpy.stop() }
        verify(exactly = 1) { magnetometerSensorCollectorSpy.stop() }

        val hasMagnetometerValues2: Boolean? = estimator.getPrivateProperty("hasMagnetometerValues")
        requireNotNull(hasMagnetometerValues2)
        assertFalse(hasMagnetometerValues2)
    }

    @Test
    fun start_whenNotRunningAndSuccess_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator(context)

        // setup spies
        val levelingEstimator: BaseLevelingEstimator<*, *>? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)
        val levelingEstimatorSpy = spyk(levelingEstimator)
        every { levelingEstimatorSpy.start() }.returns(true)
        estimator.setPrivateProperty("levelingEstimator", levelingEstimatorSpy)

        val magnetometerSensorCollector: MagnetometerSensorCollector? =
            estimator.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        every { magnetometerSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        estimator.setPrivateProperty("hasMagnetometerValues", true)

        val hasMagnetometerValues1: Boolean? = estimator.getPrivateProperty("hasMagnetometerValues")
        requireNotNull(hasMagnetometerValues1)
        assertTrue(hasMagnetometerValues1)

        // check
        assertFalse(estimator.running)

        // start
        assertTrue(estimator.start())

        verify(exactly = 1) { levelingEstimatorSpy.start() }
        verify(exactly = 1) { magnetometerSensorCollectorSpy.start() }
        verify(exactly = 0) { levelingEstimatorSpy.stop() }
        verify(exactly = 0) { magnetometerSensorCollectorSpy.stop() }

        val hasMagnetometerValues2: Boolean? = estimator.getPrivateProperty("hasMagnetometerValues")
        requireNotNull(hasMagnetometerValues2)
        assertFalse(hasMagnetometerValues2)
    }

    @Test
    fun stop_stopsLevelingEstimatorAndMagnetometerSensorCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator(context)

        // setup spies
        val levelingEstimator: BaseLevelingEstimator<*, *>? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)
        val levelingEstimatorSpy = spyk(levelingEstimator)
        every { levelingEstimatorSpy.start() }.returns(true)
        estimator.setPrivateProperty("levelingEstimator", levelingEstimatorSpy)

        val magnetometerSensorCollector: MagnetometerSensorCollector? =
            estimator.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        every { magnetometerSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        // stop
        estimator.stop()

        // check
        verify(exactly = 1) { levelingEstimatorSpy.stop() }
        verify(exactly = 1) { magnetometerSensorCollectorSpy.stop() }
        assertFalse(estimator.running)
    }

    @Test
    fun magnetometerSensorCollector_hasExpectedProperties() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator(context)

        val magnetometerSensorCollector: MagnetometerSensorCollector? =
            estimator.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)

        assertSame(context, magnetometerSensorCollector.context)
        assertEquals(estimator.magnetometerSensorType, magnetometerSensorCollector.sensorType)
        assertEquals(estimator.sensorDelay, magnetometerSensorCollector.sensorDelay)
    }

    @Test
    fun magnetometerSensorCollector_whenMeasurementAvailableAndNoHardIron_keepsSensedMagneticField() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator(context)

        val magnetometerSensorCollector: MagnetometerSensorCollector? =
            estimator.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)

        // obtain sensor collector listener
        val listener = magnetometerSensorCollector.measurementListener
        requireNotNull(listener)

        // check initial sensor values
        val sensorBx1: Double? = estimator.getPrivateProperty("sensorBx")
        requireNotNull(sensorBx1)
        assertEquals(0.0, sensorBx1, 0.0)
        val sensorBy1: Double? = estimator.getPrivateProperty("sensorBy")
        requireNotNull(sensorBy1)
        assertEquals(0.0, sensorBy1, 0.0)
        val sensorBz1: Double? = estimator.getPrivateProperty("sensorBz")
        requireNotNull(sensorBz1)
        assertEquals(0.0, sensorBz1, 0.0)
        val hasMagnetometerValues1: Boolean? = estimator.getPrivateProperty("hasMagnetometerValues")
        requireNotNull(hasMagnetometerValues1)
        assertFalse(hasMagnetometerValues1)

        // execute event
        val randomizer = UniformRandomizer()
        val microBx = randomizer.nextFloat()
        val microBy = randomizer.nextFloat()
        val microBz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onMeasurement(
            microBx,
            microBy,
            microBz,
            null,
            null,
            null,
            timestamp,
            SensorAccuracy.LOW
        )

        // check sensor values
        val sensorBx2: Double? = estimator.getPrivateProperty("sensorBx")
        requireNotNull(sensorBx2)
        assertEquals(
            MagneticFluxDensityConverter.microTeslaToTesla(microBx.toDouble()),
            sensorBx2,
            0.0
        )
        val sensorBy2: Double? = estimator.getPrivateProperty("sensorBy")
        requireNotNull(sensorBy2)
        assertEquals(
            MagneticFluxDensityConverter.microTeslaToTesla(microBy.toDouble()),
            sensorBy2,
            0.0
        )
        val sensorBz2: Double? = estimator.getPrivateProperty("sensorBz")
        requireNotNull(sensorBz2)
        assertEquals(
            MagneticFluxDensityConverter.microTeslaToTesla(microBz.toDouble()),
            sensorBz2,
            0.0
        )
        val hasMagnetometerValues2: Boolean? = estimator.getPrivateProperty("hasMagnetometerValues")
        requireNotNull(hasMagnetometerValues2)
        assertTrue(hasMagnetometerValues2)
    }

    @Test
    fun magnetometerSensorCollector_whenMeasurementAvailableAndHardIron_keepsSensedMagneticField() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator(context)

        val magnetometerSensorCollector: MagnetometerSensorCollector? =
            estimator.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)

        // obtain sensor collector listener
        val listener = magnetometerSensorCollector.measurementListener
        requireNotNull(listener)

        // check initial sensor values
        val sensorBx1: Double? = estimator.getPrivateProperty("sensorBx")
        requireNotNull(sensorBx1)
        assertEquals(0.0, sensorBx1, 0.0)
        val sensorBy1: Double? = estimator.getPrivateProperty("sensorBy")
        requireNotNull(sensorBy1)
        assertEquals(0.0, sensorBy1, 0.0)
        val sensorBz1: Double? = estimator.getPrivateProperty("sensorBz")
        requireNotNull(sensorBz1)
        assertEquals(0.0, sensorBz1, 0.0)
        val hasMagnetometerValues1: Boolean? = estimator.getPrivateProperty("hasMagnetometerValues")
        requireNotNull(hasMagnetometerValues1)
        assertFalse(hasMagnetometerValues1)

        // execute event
        val randomizer = UniformRandomizer()
        val microBx = randomizer.nextFloat()
        val microBy = randomizer.nextFloat()
        val microBz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onMeasurement(
            microBx,
            microBy,
            microBz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            SensorAccuracy.LOW
        )

        // check sensor values
        val sensorBx2: Double? = estimator.getPrivateProperty("sensorBx")
        requireNotNull(sensorBx2)
        assertEquals(
            MagneticFluxDensityConverter.microTeslaToTesla((microBx - hardIronX).toDouble()),
            sensorBx2,
            0.0
        )
        val sensorBy2: Double? = estimator.getPrivateProperty("sensorBy")
        requireNotNull(sensorBy2)
        assertEquals(
            MagneticFluxDensityConverter.microTeslaToTesla((microBy - hardIronY).toDouble()),
            sensorBy2,
            0.0
        )
        val sensorBz2: Double? = estimator.getPrivateProperty("sensorBz")
        requireNotNull(sensorBz2)
        assertEquals(
            MagneticFluxDensityConverter.microTeslaToTesla((microBz - hardIronZ).toDouble()),
            sensorBz2,
            0.0
        )
        val hasMagnetometerValues2: Boolean? = estimator.getPrivateProperty("hasMagnetometerValues")
        requireNotNull(hasMagnetometerValues2)
        assertTrue(hasMagnetometerValues2)
    }

    @Test
    fun levelingAvailableListener_whenAccurateLeveling_processesLevelingAttitude() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)
        val location = getLocation()
        val listener =
            mockk<GeomagneticAttitudeEstimator.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val estimator = GeomagneticAttitudeEstimator(
            context, location,
            useAccurateLevelingEstimator = true,
            attitudeAvailableListener = listener
        )

        estimator.setPrivateProperty("hasMagnetometerValues", true)

        val levelingEstimator: AccurateLevelingEstimator? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)

        val levelingAvailableListener = levelingEstimator.levelingAvailableListener
        requireNotNull(levelingAvailableListener)
        val attitude = Quaternion()
        levelingAvailableListener.onLevelingAvailable(levelingEstimator, attitude, null, null, null)

        verify(exactly = 1) {
            listener.onAttitudeAvailable(
                estimator,
                any(),
                any(),
                any(),
                any(),
                null
            )
        }
    }

    @Test
    fun levelingAvailableListener_whenNonAccurateLeveling_processesLevelingAttitude() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)
        val listener =
            mockk<GeomagneticAttitudeEstimator.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val estimator = GeomagneticAttitudeEstimator(
            context,
            useAccurateLevelingEstimator = false,
            attitudeAvailableListener = listener
        )

        estimator.setPrivateProperty("hasMagnetometerValues", true)

        val levelingEstimator: LevelingEstimator? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)

        val levelingAvailableListener = levelingEstimator.levelingAvailableListener
        requireNotNull(levelingAvailableListener)
        val attitude = Quaternion()
        levelingAvailableListener.onLevelingAvailable(levelingEstimator, attitude, null, null, null)

        verify(exactly = 1) {
            listener.onAttitudeAvailable(
                estimator,
                any(),
                any(),
                any(),
                any(),
                null
            )
        }
    }

    @Test
    fun processLeveling_whenNoMagnetometerValues_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val listener =
            mockk<GeomagneticAttitudeEstimator.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val estimator = GeomagneticAttitudeEstimator(context, attitudeAvailableListener = listener)

        val hasMagnetometerValues: Boolean? = estimator.getPrivateProperty("hasMagnetometerValues")
        requireNotNull(hasMagnetometerValues)
        assertFalse(hasMagnetometerValues)

        val attitude = Quaternion()
        callPrivateFunc(GeomagneticAttitudeEstimator::class, estimator, "processLeveling", attitude)

        verify { listener wasNot Called }
    }

    @Test
    fun processLeveling_whenMagnetometerValues_estimatesAttitude() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)
        val estimator = GeomagneticAttitudeEstimator(context)

        estimator.setPrivateProperty("hasMagnetometerValues", true)
        val hasMagnetometerValues: Boolean? = estimator.getPrivateProperty("hasMagnetometerValues")
        requireNotNull(hasMagnetometerValues)
        assertTrue(hasMagnetometerValues)

        // setup spies
        val displayOrientation: Quaternion? = estimator.getPrivateProperty("displayOrientation")
        requireNotNull(displayOrientation)
        val displayOrientationSpy = spyk(displayOrientation)
        estimator.setPrivateProperty("displayOrientation", displayOrientationSpy)

        val levelingAttitude: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude)
        val levelingAttitudeSpy = spyk(levelingAttitude)
        estimator.setPrivateProperty("levelingAttitude", levelingAttitudeSpy)

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)
        val fusedAttitudeSpy = spyk(fusedAttitude)
        estimator.setPrivateProperty("fusedAttitude", fusedAttitudeSpy)

        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        requireNotNull(coordinateTransformation)
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        estimator.setPrivateProperty("coordinateTransformation", coordinateTransformationSpy)

        val attitude = spyk(Quaternion())
        callPrivateFunc(GeomagneticAttitudeEstimator::class, estimator, "processLeveling", attitude)

        verify(exactly = 1) { displayOrientationSpy.setFromEulerAngles(0.0, 0.0, 0.0) }
        verify(exactly = 1) { attitude.inverse(levelingAttitudeSpy) }
        verify(exactly = 1) { levelingAttitudeSpy.toEulerAngles(any()) }
        verify(exactly = 1) { fusedAttitudeSpy.setFromEulerAngles(any(), any(), any()) }
        verify(exactly = 1) { fusedAttitudeSpy.combine(displayOrientationSpy) }
        verify(exactly = 3) { fusedAttitudeSpy.normalize() }
        verify(exactly = 1) { fusedAttitudeSpy.inverse() }
        verify { coordinateTransformationSpy wasNot Called }
        verify(exactly = 1) { fusedAttitudeSpy.toEulerAngles(any()) }
    }

    @Test
    fun processLeveling_whenCoordinateTransformationEstimated_estimatesAttitude() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)
        val listener =
            mockk<GeomagneticAttitudeEstimator.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val estimator = GeomagneticAttitudeEstimator(
            context,
            estimateCoordinateTransformation = true,
            attitudeAvailableListener = listener
        )

        estimator.setPrivateProperty("hasMagnetometerValues", true)
        val hasMagnetometerValues: Boolean? = estimator.getPrivateProperty("hasMagnetometerValues")
        requireNotNull(hasMagnetometerValues)
        assertTrue(hasMagnetometerValues)

        // setup spies
        val displayOrientation: Quaternion? = estimator.getPrivateProperty("displayOrientation")
        requireNotNull(displayOrientation)
        val displayOrientationSpy = spyk(displayOrientation)
        estimator.setPrivateProperty("displayOrientation", displayOrientationSpy)

        val levelingAttitude: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude)
        val levelingAttitudeSpy = spyk(levelingAttitude)
        estimator.setPrivateProperty("levelingAttitude", levelingAttitudeSpy)

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)
        val fusedAttitudeSpy = spyk(fusedAttitude)
        estimator.setPrivateProperty("fusedAttitude", fusedAttitudeSpy)

        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        requireNotNull(coordinateTransformation)
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        estimator.setPrivateProperty("coordinateTransformation", coordinateTransformationSpy)

        val attitude = spyk(Quaternion())
        callPrivateFunc(GeomagneticAttitudeEstimator::class, estimator, "processLeveling", attitude)

        verify(exactly = 1) { displayOrientationSpy.setFromEulerAngles(0.0, 0.0, 0.0) }
        verify(exactly = 1) { attitude.inverse(levelingAttitudeSpy) }
        verify(exactly = 1) { levelingAttitudeSpy.toEulerAngles(any()) }
        verify(exactly = 1) { fusedAttitudeSpy.setFromEulerAngles(any(), any(), any()) }
        verify(exactly = 1) { fusedAttitudeSpy.combine(displayOrientationSpy) }
        verify(exactly = 3) { fusedAttitudeSpy.normalize() }
        verify(exactly = 1) { fusedAttitudeSpy.inverse() }
        verify(exactly = 1) { fusedAttitudeSpy.asInhomogeneousMatrix(any()) }
        verify(exactly = 1) { coordinateTransformationSpy.fromRotation(fusedAttitudeSpy) }
        verify(exactly = 1) { fusedAttitudeSpy.toEulerAngles(any()) }

        verify(exactly = 1) {
            listener.onAttitudeAvailable(
                estimator,
                fusedAttitudeSpy,
                any(),
                any(),
                any(),
                coordinateTransformationSpy
            )
        }
    }

    @Test
    fun processLeveling_whenEulerAnglesNotEstimated_estimatesAttitude() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)
        val listener =
            mockk<GeomagneticAttitudeEstimator.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val estimator = GeomagneticAttitudeEstimator(
            context,
            estimateDisplayEulerAngles = false,
            attitudeAvailableListener = listener
        )

        estimator.setPrivateProperty("hasMagnetometerValues", true)
        val hasMagnetometerValues: Boolean? = estimator.getPrivateProperty("hasMagnetometerValues")
        requireNotNull(hasMagnetometerValues)
        assertTrue(hasMagnetometerValues)

        // setup spies
        val displayOrientation: Quaternion? = estimator.getPrivateProperty("displayOrientation")
        requireNotNull(displayOrientation)
        val displayOrientationSpy = spyk(displayOrientation)
        estimator.setPrivateProperty("displayOrientation", displayOrientationSpy)

        val levelingAttitude: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude)
        val levelingAttitudeSpy = spyk(levelingAttitude)
        estimator.setPrivateProperty("levelingAttitude", levelingAttitudeSpy)

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)
        val fusedAttitudeSpy = spyk(fusedAttitude)
        estimator.setPrivateProperty("fusedAttitude", fusedAttitudeSpy)

        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        requireNotNull(coordinateTransformation)
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        estimator.setPrivateProperty("coordinateTransformation", coordinateTransformationSpy)

        val attitude = spyk(Quaternion())
        callPrivateFunc(GeomagneticAttitudeEstimator::class, estimator, "processLeveling", attitude)

        verify(exactly = 1) { displayOrientationSpy.setFromEulerAngles(0.0, 0.0, 0.0) }
        verify(exactly = 1) { attitude.inverse(levelingAttitudeSpy) }
        verify(exactly = 1) { levelingAttitudeSpy.toEulerAngles(any()) }
        verify(exactly = 1) { fusedAttitudeSpy.setFromEulerAngles(any(), any(), any()) }
        verify(exactly = 1) { fusedAttitudeSpy.combine(displayOrientationSpy) }
        verify(exactly = 3) { fusedAttitudeSpy.normalize() }
        verify(exactly = 1) { fusedAttitudeSpy.inverse() }
        verify(exactly = 0) { fusedAttitudeSpy.asInhomogeneousMatrix(any()) }
        verify { coordinateTransformationSpy wasNot Called }
        verify(exactly = 0) { fusedAttitudeSpy.toEulerAngles(any()) }

        verify(exactly = 1) {
            listener.onAttitudeAvailable(
                estimator,
                fusedAttitudeSpy,
                null,
                null,
                null,
                null
            )
        }
    }

    @Test
    fun processLeveling_whenNoMagneticModel_estimatesAttitude() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)
        val estimator = GeomagneticAttitudeEstimator(context)

        assertNull(estimator.worldMagneticModel)
        assertFalse(estimator.useWorldMagneticModel)

        estimator.setPrivateProperty("hasMagnetometerValues", true)
        val hasMagnetometerValues: Boolean? = estimator.getPrivateProperty("hasMagnetometerValues")
        requireNotNull(hasMagnetometerValues)
        assertTrue(hasMagnetometerValues)

        // setup spies
        val displayOrientation: Quaternion? = estimator.getPrivateProperty("displayOrientation")
        requireNotNull(displayOrientation)
        val displayOrientationSpy = spyk(displayOrientation)
        estimator.setPrivateProperty("displayOrientation", displayOrientationSpy)

        val levelingAttitude: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude)
        val levelingAttitudeSpy = spyk(levelingAttitude)
        estimator.setPrivateProperty("levelingAttitude", levelingAttitudeSpy)

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)
        val fusedAttitudeSpy = spyk(fusedAttitude)
        estimator.setPrivateProperty("fusedAttitude", fusedAttitudeSpy)

        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        requireNotNull(coordinateTransformation)
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        estimator.setPrivateProperty("coordinateTransformation", coordinateTransformationSpy)

        val attitude = spyk(Quaternion())
        callPrivateFunc(GeomagneticAttitudeEstimator::class, estimator, "processLeveling", attitude)

        verify(exactly = 1) { displayOrientationSpy.setFromEulerAngles(0.0, 0.0, 0.0) }
        verify(exactly = 1) { attitude.inverse(levelingAttitudeSpy) }
        verify(exactly = 1) { levelingAttitudeSpy.toEulerAngles(any()) }
        verify(exactly = 1) { fusedAttitudeSpy.setFromEulerAngles(any(), any(), any()) }
        verify(exactly = 1) { fusedAttitudeSpy.combine(displayOrientationSpy) }
        verify(exactly = 3) { fusedAttitudeSpy.normalize() }
        verify(exactly = 1) { fusedAttitudeSpy.inverse() }
        verify { coordinateTransformationSpy wasNot Called }
        verify(exactly = 1) { fusedAttitudeSpy.toEulerAngles(any()) }
    }

    @Test
    fun processLeveling_whenMagneticModel_estimatesAttitude() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)
        val location = getLocation()
        val worldMagneticModel = WorldMagneticModel()
        val estimator =
            GeomagneticAttitudeEstimator(
                context,
                location,
                worldMagneticModel = worldMagneticModel,
                useWorldMagneticModel = true
            )

        assertSame(worldMagneticModel, estimator.worldMagneticModel)
        assertTrue(estimator.useWorldMagneticModel)

        estimator.setPrivateProperty("hasMagnetometerValues", true)
        val hasMagnetometerValues: Boolean? = estimator.getPrivateProperty("hasMagnetometerValues")
        requireNotNull(hasMagnetometerValues)
        assertTrue(hasMagnetometerValues)

        // setup spies
        val displayOrientation: Quaternion? = estimator.getPrivateProperty("displayOrientation")
        requireNotNull(displayOrientation)
        val displayOrientationSpy = spyk(displayOrientation)
        estimator.setPrivateProperty("displayOrientation", displayOrientationSpy)

        val levelingAttitude: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude)
        val levelingAttitudeSpy = spyk(levelingAttitude)
        estimator.setPrivateProperty("levelingAttitude", levelingAttitudeSpy)

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)
        val fusedAttitudeSpy = spyk(fusedAttitude)
        estimator.setPrivateProperty("fusedAttitude", fusedAttitudeSpy)

        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        requireNotNull(coordinateTransformation)
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        estimator.setPrivateProperty("coordinateTransformation", coordinateTransformationSpy)

        val attitude = spyk(Quaternion())
        callPrivateFunc(GeomagneticAttitudeEstimator::class, estimator, "processLeveling", attitude)

        verify(exactly = 1) { displayOrientationSpy.setFromEulerAngles(0.0, 0.0, 0.0) }
        verify(exactly = 1) { attitude.inverse(levelingAttitudeSpy) }
        verify(exactly = 1) { levelingAttitudeSpy.toEulerAngles(any()) }
        verify(exactly = 1) { fusedAttitudeSpy.setFromEulerAngles(any(), any(), any()) }
        verify(exactly = 1) { fusedAttitudeSpy.combine(displayOrientationSpy) }
        verify(exactly = 3) { fusedAttitudeSpy.normalize() }
        verify(exactly = 1) { fusedAttitudeSpy.inverse() }
        verify { coordinateTransformationSpy wasNot Called }
        verify(exactly = 1) { fusedAttitudeSpy.toEulerAngles(any()) }
    }

    @Test
    fun processLeveling_whenIgnoreDisplayOrientation_estimatesAttitude() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)
        val listener =
            mockk<GeomagneticAttitudeEstimator.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val estimator = GeomagneticAttitudeEstimator(
            context,
            ignoreDisplayOrientation = true,
            attitudeAvailableListener = listener
        )

        estimator.setPrivateProperty("hasMagnetometerValues", true)
        val hasMagnetometerValues: Boolean? = estimator.getPrivateProperty("hasMagnetometerValues")
        requireNotNull(hasMagnetometerValues)
        assertTrue(hasMagnetometerValues)

        // setup spies
        val displayOrientation: Quaternion? = estimator.getPrivateProperty("displayOrientation")
        requireNotNull(displayOrientation)
        val displayOrientationSpy = spyk(displayOrientation)
        estimator.setPrivateProperty("displayOrientation", displayOrientationSpy)

        val levelingAttitude: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude)
        val levelingAttitudeSpy = spyk(levelingAttitude)
        estimator.setPrivateProperty("levelingAttitude", levelingAttitudeSpy)

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)
        val fusedAttitudeSpy = spyk(fusedAttitude)
        estimator.setPrivateProperty("fusedAttitude", fusedAttitudeSpy)

        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        requireNotNull(coordinateTransformation)
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        estimator.setPrivateProperty("coordinateTransformation", coordinateTransformationSpy)

        val attitude = spyk(Quaternion())
        callPrivateFunc(GeomagneticAttitudeEstimator::class, estimator, "processLeveling", attitude)

        verify(exactly = 0) { displayOrientationSpy.setFromEulerAngles(0.0, 0.0, 0.0) }
        verify(exactly = 1) { attitude.inverse(levelingAttitudeSpy) }
        verify(exactly = 1) { levelingAttitudeSpy.toEulerAngles(any()) }
        verify(exactly = 1) { fusedAttitudeSpy.setFromEulerAngles(any(), any(), any()) }
        verify(exactly = 0) { fusedAttitudeSpy.combine(displayOrientationSpy) }
        verify(exactly = 3) { fusedAttitudeSpy.normalize() }
        verify(exactly = 1) { fusedAttitudeSpy.inverse() }
        verify(exactly = 0) { fusedAttitudeSpy.asInhomogeneousMatrix(any()) }
        verify { coordinateTransformationSpy wasNot Called }
        verify(exactly = 1) { fusedAttitudeSpy.toEulerAngles(any()) }

        verify(exactly = 1) {
            listener.onAttitudeAvailable(
                estimator,
                fusedAttitudeSpy,
                any(),
                any(),
                any(),
                null
            )
        }
    }

    private companion object {
        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 4000.0

        fun getLocation(): Location {
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

            val location = mockk<Location>()
            every { location.latitude }.returns(latitudeDegrees)
            every { location.longitude }.returns(longitudeDegrees)
            every { location.altitude }.returns(height)

            return location
        }
    }
}