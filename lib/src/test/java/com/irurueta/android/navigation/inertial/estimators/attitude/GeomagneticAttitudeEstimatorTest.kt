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

package com.irurueta.android.navigation.inertial.estimators.attitude

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorManager
import android.location.Location
import android.os.SystemClock
import com.irurueta.android.navigation.inertial.collectors.AccelerometerAndMagnetometerSyncedSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GravityAndMagnetometerSyncedSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerAndMagnetometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.GravityAndMagnetometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorType
import com.irurueta.android.navigation.inertial.processors.attitude.AccelerometerGeomagneticAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.attitude.GeomagneticAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.filters.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.processors.filters.MeanAveragingFilter
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import io.mockk.Called
import io.mockk.every
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import io.mockk.justRun
import io.mockk.mockkStatic
import io.mockk.spyk
import io.mockk.verify
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test
import java.util.Date

class GeomagneticAttitudeEstimatorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var attitudeAvailableListener:
            GeomagneticAttitudeEstimator.OnAttitudeAvailableListener

    @MockK(relaxUnitFun = true)
    private lateinit var accuracyChangedListener:
            GeomagneticAttitudeEstimator.OnAccuracyChangedListener

    @MockK
    private lateinit var location: Location

    @MockK
    private lateinit var accelerometerSensor: Sensor

    @MockK
    private lateinit var gravitySensor: Sensor

    @MockK
    private lateinit var magnetometerSensor: Sensor

    @MockK
    private lateinit var context: Context

    @MockK
    private lateinit var sensorManager: SensorManager

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val estimator = GeomagneticAttitudeEstimator(context)

        // check
        assertSame(context, estimator.context)
        assertNull(estimator.location)
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertTrue(estimator.useAccelerometer)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            estimator.magnetometerSensorType
        )
        assertNotNull(estimator.accelerometerAveragingFilter)
        assertTrue(estimator.accelerometerAveragingFilter is LowPassAveragingFilter)
        assertNull(estimator.worldMagneticModel)
        assertFalse(estimator.useWorldMagneticModel)
        assertNull(estimator.timestamp)
        assertFalse(estimator.useWorldMagneticModel)
        assertFalse(estimator.useAccurateLevelingEstimation)
        assertFalse(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateEulerAngles)
        assertNull(estimator.attitudeAvailableListener)
        assertNull(estimator.accuracyChangedListener)
        assertTrue(estimator.adjustGravityNorm)
    }

    @Test
    fun constructor_whenAllProperties_setsDefaultValues() {
        val location = getLocation()
        val worldMagneticModel = WorldMagneticModel()
        val timestamp = Date()
        val accelerometerAveragingFilter = MeanAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()
        val estimator = GeomagneticAttitudeEstimator(
            context,
            location,
            SensorDelay.NORMAL,
            useAccelerometer = false,
            AccelerometerSensorType.ACCELEROMETER,
            MagnetometerSensorType.MAGNETOMETER,
            accelerometerAveragingFilter,
            worldMagneticModel,
            timestamp,
            useWorldMagneticModel = true,
            useAccurateLevelingEstimation = true,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeAvailableListener,
            accuracyChangedListener = accuracyChangedListener
        )

        // check
        assertSame(context, estimator.context)
        assertSame(location, estimator.location)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertFalse(estimator.useAccelerometer)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            estimator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            estimator.magnetometerSensorType
        )
        assertSame(accelerometerAveragingFilter, estimator.accelerometerAveragingFilter)
        assertSame(worldMagneticModel, estimator.worldMagneticModel)
        assertEquals(timestamp, estimator.timestamp)
        assertTrue(estimator.useWorldMagneticModel)
        assertTrue(estimator.useAccurateLevelingEstimation)
        assertTrue(estimator.estimateCoordinateTransformation)
        assertFalse(estimator.estimateEulerAngles)
        assertSame(attitudeAvailableListener, estimator.attitudeAvailableListener)
        assertSame(accuracyChangedListener, estimator.accuracyChangedListener)
        assertTrue(estimator.adjustGravityNorm)
    }

    @Test
    fun constructor_whenAccurateLevelingEnabledAndMissingLocation_throwsIllegalStateException() {
        assertThrows(IllegalStateException::class.java) {
            GeomagneticAttitudeEstimator(context, useAccurateLevelingEstimation = true)
        }
    }

    @Test
    fun timestamp_setsExpectedValue() {
        val estimator = GeomagneticAttitudeEstimator(context)

        val accelerometerProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: GeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        // check default value
        assertNull(estimator.timestamp)

        val timestamp = Date()
        estimator.timestamp = timestamp

        // check
        assertSame(timestamp, estimator.timestamp)
        assertSame(timestamp, accelerometerProcessor.currentDate)
        assertSame(timestamp, gravityProcessor.currentDate)
    }

    @Test
    fun attitudeAvailableListener_setsExpectedValue() {
        val estimator = GeomagneticAttitudeEstimator(context)

        // check default value
        assertNull(estimator.attitudeAvailableListener)

        // set new value
        estimator.attitudeAvailableListener = attitudeAvailableListener

        // check
        assertSame(attitudeAvailableListener, estimator.attitudeAvailableListener)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val estimator = GeomagneticAttitudeEstimator(context)

        // check default value
        assertNull(estimator.accuracyChangedListener)

        // set new value
        estimator.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, estimator.accuracyChangedListener)
    }

    @Test
    fun location_whenNotRunning_setsExpectedValue() {
        val estimator = GeomagneticAttitudeEstimator(context)

        val accelerometerProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: GeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        // check default value
        assertNull(estimator.location)
        assertFalse(estimator.running)

        // set new value
        val location = getLocation()
        estimator.location = location

        // check
        assertSame(location, estimator.location)
        assertSame(location, accelerometerProcessor.location)
        assertSame(location, gravityProcessor.location)

        // set new value
        estimator.location = null

        // check
        assertNull(estimator.location)
        assertNull(accelerometerProcessor.location)
        assertNull(gravityProcessor.location)
    }

    @Test
    fun location_whenRunningAndNotNull_setsExpectedValue() {
        val estimator = GeomagneticAttitudeEstimator(context)

        val accelerometerProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: GeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        assertFalse(estimator.running)
        assertNull(estimator.location)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        val location = getLocation()
        estimator.location = location

        // check
        assertSame(location, estimator.location)
        assertSame(location, accelerometerProcessor.location)
        assertSame(location, gravityProcessor.location)
    }

    @Test
    fun location_whenRunningAndNull_throwsIllegalStateException() {
        val location = getLocation()
        val estimator = GeomagneticAttitudeEstimator(context, location)

        assertFalse(estimator.running)
        assertSame(location, estimator.location)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        // set new value
        assertThrows(IllegalStateException::class.java) {
            estimator.location = null
        }
    }

    @Test
    fun adjustGravityNorm_whenNotRunning_setsExpectedValue() {
        val estimator = GeomagneticAttitudeEstimator(context)

        // check default value
        assertTrue(estimator.adjustGravityNorm)

        val gravityProcessor: GeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        assertTrue(gravityProcessor.adjustGravityNorm)

        val accelerometerProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        assertTrue(accelerometerProcessor.adjustGravityNorm)

        // set new value
        estimator.adjustGravityNorm = false

        // check
        assertFalse(estimator.adjustGravityNorm)
        assertFalse(gravityProcessor.adjustGravityNorm)
        assertFalse(accelerometerProcessor.adjustGravityNorm)
    }

    @Test
    fun useAccurateLevelingEstimation_whenTrueValueNotRunningAndLocationAvailable_setsExpectedValue() {
        val location = getLocation()
        val estimator = GeomagneticAttitudeEstimator(context, location)

        val accelerometerProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: GeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        // check default value
        assertFalse(estimator.useAccurateLevelingEstimation)
        assertFalse(estimator.running)
        assertSame(location, estimator.location)

        // set new value
        estimator.useAccurateLevelingEstimation = true

        // check
        assertTrue(estimator.useAccurateLevelingEstimation)
        assertTrue(accelerometerProcessor.useAccurateLevelingProcessor)
        assertTrue(gravityProcessor.useAccurateLevelingProcessor)
    }

    @Test
    fun useAccurateLevelingEstimation_whenRunning_throwsIllegalStateException() {
        val estimator = GeomagneticAttitudeEstimator(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        assertThrows(IllegalStateException::class.java) {
            estimator.useAccurateLevelingEstimation = true
        }
    }

    @Test
    fun useAccurateLevelingEstimation_whenNotRunningAndNoLocation_throwsIllegalStateException() {
        val estimator = GeomagneticAttitudeEstimator(context)

        // check default values
        assertFalse(estimator.running)
        assertNull(estimator.location)
        assertFalse(estimator.useAccurateLevelingEstimation)

        // set new value
        assertThrows(IllegalStateException::class.java) {
            estimator.useAccurateLevelingEstimation = true
        }
    }

    @Test
    fun useAccurateLevelingEstimation_whenFalseValueNotRunningAndNoLocation_setsExpectedValue() {
        val estimator = GeomagneticAttitudeEstimator(context)

        val accelerometerProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: GeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        // check default values
        assertFalse(estimator.running)
        assertNull(estimator.location)
        assertFalse(estimator.useAccurateLevelingEstimation)

        // set new value
        estimator.useAccurateLevelingEstimation = false

        // check
        assertFalse(estimator.useAccurateLevelingEstimation)
        assertFalse(accelerometerProcessor.useAccurateLevelingProcessor)
        assertFalse(gravityProcessor.useAccurateLevelingProcessor)
    }

    @Test
    fun worldMagneticModel_whenNotRunningAndNotUseWorldMagneticModel_setsExpectedValue() {
        val estimator = GeomagneticAttitudeEstimator(context)

        val accelerometerProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: GeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        // check default values
        assertNull(estimator.worldMagneticModel)
        assertFalse(estimator.running)
        assertFalse(estimator.useWorldMagneticModel)

        // set new value
        val worldMagneticModel = WorldMagneticModel()
        estimator.worldMagneticModel = worldMagneticModel

        // check
        assertSame(worldMagneticModel, estimator.worldMagneticModel)
        assertSame(worldMagneticModel, accelerometerProcessor.worldMagneticModel)
        assertSame(worldMagneticModel, gravityProcessor.worldMagneticModel)

        // set new value
        estimator.worldMagneticModel = null

        // check
        assertNull(estimator.worldMagneticModel)
        assertNull(accelerometerProcessor.worldMagneticModel)
        assertNull(gravityProcessor.worldMagneticModel)
    }

    @Test
    fun worldMagneticModel_whenNotRunningAndUseWorldMagneticModel_setsExpectedValue() {
        val estimator = GeomagneticAttitudeEstimator(context, useWorldMagneticModel = true)

        val accelerometerProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: GeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        // check default values
        assertNull(estimator.worldMagneticModel)
        assertFalse(estimator.running)
        assertTrue(estimator.useWorldMagneticModel)

        // set new value
        val worldMagneticModel = WorldMagneticModel()
        estimator.worldMagneticModel = worldMagneticModel

        // check
        assertSame(worldMagneticModel, estimator.worldMagneticModel)
        assertSame(worldMagneticModel, accelerometerProcessor.worldMagneticModel)
        assertSame(worldMagneticModel, gravityProcessor.worldMagneticModel)

        // set new value
        estimator.worldMagneticModel = null

        // check
        assertNull(estimator.worldMagneticModel)
        assertNull(accelerometerProcessor.worldMagneticModel)
        assertNull(gravityProcessor.worldMagneticModel)
    }

    @Test
    fun worldMagneticModel_whenRunning_throwsIllegalStateException() {
        val estimator = GeomagneticAttitudeEstimator(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        // set new value
        val worldMagneticModel = WorldMagneticModel()
        assertThrows(IllegalStateException::class.java) {
            estimator.worldMagneticModel = worldMagneticModel
        }
    }

    @Test
    fun useWorldMagneticModel_whenNotRunning_setsExpectedValue() {
        val estimator = GeomagneticAttitudeEstimator(context)

        val accelerometerProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: GeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        // check default values
        assertFalse(estimator.running)
        assertFalse(estimator.useWorldMagneticModel)

        // set new value
        estimator.useWorldMagneticModel = true

        // check
        assertTrue(estimator.useWorldMagneticModel)
        assertTrue(accelerometerProcessor.useWorldMagneticModel)
        assertTrue(gravityProcessor.useWorldMagneticModel)

        // set new value
        estimator.useWorldMagneticModel = false

        // check
        assertFalse(estimator.useWorldMagneticModel)
        assertFalse(accelerometerProcessor.useWorldMagneticModel)
        assertFalse(gravityProcessor.useWorldMagneticModel)
    }

    @Test
    fun useWorldMagneticModel_whenRunning_throwsIllegalStateException() {
        val estimator = GeomagneticAttitudeEstimator(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        // set new value
        assertThrows(IllegalStateException::class.java) {
            estimator.useWorldMagneticModel = true
        }
    }

    @Test
    fun start_whenRunningAndUseAccelerometer_throwsIllegalStateException() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            val estimator = GeomagneticAttitudeEstimator(context)

            // set as running
            estimator.setPrivateProperty("running", true)
            assertTrue(estimator.running)

            assertThrows(IllegalStateException::class.java) {
                estimator.start()
            }
        }
    }

    @Test
    fun start_whenNotRunningAndUseAccelerometer_resetsAndStartsSyncedCollector() {
        val timestamp = System.nanoTime()
        val estimator = GeomagneticAttitudeEstimator(context, useAccelerometer = true)

        // setup spies
        val accelerometerProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val gravityProcessor: GeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val accelerometerAndMagnetometerCollector: AccelerometerAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerAndMagnetometerCollector")
        requireNotNull(accelerometerAndMagnetometerCollector)
        val accelerometerAndMagnetometerCollectorSpy = spyk(accelerometerAndMagnetometerCollector)
        every { accelerometerAndMagnetometerCollectorSpy.start(timestamp) }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerAndMagnetometerCollector",
            accelerometerAndMagnetometerCollectorSpy
        )

        val gravityAndMagnetometerCollector: GravityAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityAndMagnetometerCollector")
        requireNotNull(gravityAndMagnetometerCollector)
        val gravityAndMagnetometerCollectorSpy = spyk(gravityAndMagnetometerCollector)
        estimator.setPrivateProperty(
            "gravityAndMagnetometerCollector",
            gravityAndMagnetometerCollectorSpy
        )

        assertFalse(estimator.running)

        assertTrue(estimator.start(timestamp))

        verify(exactly = 1) { accelerometerProcessorSpy.reset() }
        verify(exactly = 1) { accelerometerAndMagnetometerCollectorSpy.start(timestamp) }
        verify { gravityProcessorSpy wasNot Called }
        verify { gravityAndMagnetometerCollectorSpy wasNot Called }
    }

    @Test
    fun start_whenNotRunningAndAccelerometerNotUsed_resetsAndStartsSyncer() {
        val timestamp = System.nanoTime()
        val estimator = GeomagneticAttitudeEstimator(context, useAccelerometer = false)

        // setup spies
        val accelerometerProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val gravityProcessor: GeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val accelerometerAndMagnetometerCollector: AccelerometerAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerAndMagnetometerCollector")
        requireNotNull(accelerometerAndMagnetometerCollector)
        val accelerometerAndMagnetometerCollectorSpy = spyk(accelerometerAndMagnetometerCollector)
        estimator.setPrivateProperty(
            "accelerometerAndMagnetometerCollector",
            accelerometerAndMagnetometerCollectorSpy
        )

        val gravityAndMagnetometerCollector: GravityAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityAndMagnetometerCollector")
        requireNotNull(gravityAndMagnetometerCollector)
        val gravityAndMagnetometerCollectorSpy = spyk(gravityAndMagnetometerCollector)
        every { gravityAndMagnetometerCollectorSpy.start(timestamp) }.returns(true)
        estimator.setPrivateProperty(
            "gravityAndMagnetometerCollector",
            gravityAndMagnetometerCollectorSpy
        )

        assertFalse(estimator.running)

        assertTrue(estimator.start(timestamp))

        verify(exactly = 1) { gravityProcessorSpy.reset() }
        verify(exactly = 1) { gravityAndMagnetometerCollectorSpy.start(timestamp) }
        verify { accelerometerProcessorSpy wasNot Called }
        verify { accelerometerAndMagnetometerCollectorSpy wasNot Called }
    }

    @Test
    fun stop_stopsInternalSyncers() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }
            .returns(accelerometerSensor)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }
            .returns(gravitySensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }
            .returns(magnetometerSensor)
        justRun { sensorManager.unregisterListener(any(), any<Sensor>()) }

        val estimator = GeomagneticAttitudeEstimator(context)

        // setup spies
        val accelerometerAndMagnetometerCollector: AccelerometerAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerAndMagnetometerCollector")
        requireNotNull(accelerometerAndMagnetometerCollector)
        val accelerometerAndMagnetometerCollectorSpy = spyk(accelerometerAndMagnetometerCollector)
        estimator.setPrivateProperty(
            "accelerometerAndMagnetometerCollector",
            accelerometerAndMagnetometerCollectorSpy
        )

        val gravityAndMagnetometerCollector: GravityAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityAndMagnetometerCollector")
        requireNotNull(gravityAndMagnetometerCollector)
        val gravityAndMagnetometerSyncerSpy = spyk(gravityAndMagnetometerCollector)
        estimator.setPrivateProperty(
            "gravityAndMagnetometerCollector",
            gravityAndMagnetometerSyncerSpy
        )

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        // stop
        estimator.stop()

        // check
        assertFalse(estimator.running)
        verify(exactly = 1) { accelerometerAndMagnetometerCollectorSpy.stop() }
        verify(exactly = 1) { gravityAndMagnetometerSyncerSpy.stop() }
    }

    @Test
    fun gravitySyncedCollector_whenAccuracyChangedAndNoListener_makesNoAction() {
        val estimator = GeomagneticAttitudeEstimator(context)

        val gravityAndMagnetometerCollector: GravityAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityAndMagnetometerCollector")
        requireNotNull(gravityAndMagnetometerCollector)

        val listener = gravityAndMagnetometerCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            gravityAndMagnetometerCollector,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )
    }

    @Test
    fun gravitySyncedCollector_whenAccuracyChangedAndListener_notifies() {
        val estimator = GeomagneticAttitudeEstimator(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val gravityAndMagnetometerCollector: GravityAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityAndMagnetometerCollector")
        requireNotNull(gravityAndMagnetometerCollector)

        val listener = gravityAndMagnetometerCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            gravityAndMagnetometerCollector,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )

        // check
        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                estimator,
                SensorType.GRAVITY,
                SensorAccuracy.MEDIUM
            )
        }
    }

    @Test
    fun gravitySyncedCollector_whenSyncedMeasurementAndNotProcessed_makesNoAction() {
        val estimator = GeomagneticAttitudeEstimator(
            context,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val gravityAndMagnetometerCollector: GravityAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityAndMagnetometerCollector")
        requireNotNull(gravityAndMagnetometerCollector)

        val measurement = GravityAndMagnetometerSyncedSensorMeasurement()

        val gravityProcessor: GeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(measurement) }.returns(false)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val listener = gravityAndMagnetometerCollector.measurementListener
        requireNotNull(listener)
        listener.onMeasurement(gravityAndMagnetometerCollector, measurement)

        verify { attitudeAvailableListener wasNot Called }
    }

    @Test
    fun gravitySyncedCollector_whenSyncedMeasurementProcessedAndNoListener_updatesFusedAttitude() {
        val estimator = GeomagneticAttitudeEstimator(context)

        val gravityAndMagnetometerCollector: GravityAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityAndMagnetometerCollector")
        requireNotNull(gravityAndMagnetometerCollector)

        val measurement = GravityAndMagnetometerSyncedSensorMeasurement()
        val fusedAttitude1 = getAttitude()

        val gravityProcessor: GeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(measurement) }.returns(true)
        every { gravityProcessorSpy.fusedAttitude }.returns(fusedAttitude1)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val listener = gravityAndMagnetometerCollector.measurementListener
        requireNotNull(listener)
        listener.onMeasurement(gravityAndMagnetometerCollector, measurement)

        val fusedAttitude2: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude2)
        assertEquals(fusedAttitude1, fusedAttitude2)
    }

    @Test
    fun gravitySyncedCollector_whenSyncedMeasurementProcessedEstimateCoordinateTransformationAndEulerAnglesDisableAndListener_updatesFusedAttitudeAndNotifies() {
        val estimator = GeomagneticAttitudeEstimator(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val gravityAndMagnetometerCollector: GravityAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityAndMagnetometerCollector")
        requireNotNull(gravityAndMagnetometerCollector)

        val timestamp = System.nanoTime()
        val measurement = GravityAndMagnetometerSyncedSensorMeasurement(timestamp = timestamp)
        val fusedAttitude1 = getAttitude()

        val gravityProcessor: GeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(measurement) }.returns(true)
        every { gravityProcessorSpy.fusedAttitude }.returns(fusedAttitude1)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val listener = gravityAndMagnetometerCollector.measurementListener
        requireNotNull(listener)
        listener.onMeasurement(gravityAndMagnetometerCollector, measurement)

        val fusedAttitude2: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude2)
        assertEquals(fusedAttitude1, fusedAttitude2)

        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                fusedAttitude2,
                timestamp,
                null,
                null,
                null,
                null
            )
        }
    }

    @Test
    fun gravitySyncedCollector_whenSyncedMeasurementProcessedEstimateCoordinateTransformationAndEulerAnglesEnableAndListener_updatesFusedAttitudeAndNotifies() {
        val estimator = GeomagneticAttitudeEstimator(
            context,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = true,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val gravityAndMagnetometerCollector: GravityAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityAndMagnetometerCollector")
        requireNotNull(gravityAndMagnetometerCollector)

        val timestamp = System.nanoTime()
        val measurement = GravityAndMagnetometerSyncedSensorMeasurement(timestamp = timestamp)
        val fusedAttitude1 = getAttitude()

        val gravityProcessor: GeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(measurement) }.returns(true)
        every { gravityProcessorSpy.fusedAttitude }.returns(fusedAttitude1)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val listener = gravityAndMagnetometerCollector.measurementListener
        requireNotNull(listener)
        listener.onMeasurement(gravityAndMagnetometerCollector, measurement)

        val fusedAttitude2: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude2)
        assertEquals(fusedAttitude1, fusedAttitude2)

        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        requireNotNull(coordinateTransformation)
        assertEquals(
            CoordinateTransformation(
                fusedAttitude1,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            ), coordinateTransformation
        )

        val eulerAngles1: DoubleArray? = estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles1)
        val eulerAngles2 = fusedAttitude1.toEulerAngles()
        assertArrayEquals(eulerAngles1, eulerAngles2, 0.0)

        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                fusedAttitude2,
                timestamp,
                eulerAngles2[0],
                eulerAngles2[1],
                eulerAngles2[2],
                coordinateTransformation
            )
        }
    }

    @Test
    fun accelerometerCollector_whenAccuracyChangedAndNoListener_makesNoAction() {
        val estimator = GeomagneticAttitudeEstimator(context)

        val accelerometerAndMagnetometerCollector: AccelerometerAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerAndMagnetometerCollector")
        requireNotNull(accelerometerAndMagnetometerCollector)

        val listener = accelerometerAndMagnetometerCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            accelerometerAndMagnetometerCollector,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )
    }

    @Test
    fun accelerometerCollector_whenAccuracyChangedAndListener_notifies() {
        val estimator = GeomagneticAttitudeEstimator(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val accelerometerAndMagnetometerCollector: AccelerometerAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerAndMagnetometerCollector")
        requireNotNull(accelerometerAndMagnetometerCollector)

        val listener = accelerometerAndMagnetometerCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            accelerometerAndMagnetometerCollector,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )

        // check
        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                estimator,
                SensorType.GRAVITY,
                SensorAccuracy.MEDIUM
            )
        }
    }

    @Test
    fun accelerometerCollector_whenSyncedMeasurementAndNotProcessed_makesNoAction() {
        val estimator = GeomagneticAttitudeEstimator(
            context,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val accelerometerAndMagnetometerCollector: AccelerometerAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerAndMagnetometerCollector")
        requireNotNull(accelerometerAndMagnetometerCollector)

        val measurement = AccelerometerAndMagnetometerSyncedSensorMeasurement()

        val accelerometerProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        every { accelerometerProcessorSpy.process(measurement) }.returns(false)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val listener = accelerometerAndMagnetometerCollector.measurementListener
        requireNotNull(listener)
        listener.onMeasurement(accelerometerAndMagnetometerCollector, measurement)

        verify { attitudeAvailableListener wasNot Called }
    }

    @Test
    fun accelerometerCollector_whenSyncedMeasurementProcessedAndNoListener_updatesFusedAttitude() {
        val estimator = GeomagneticAttitudeEstimator(context)

        val accelerometerAndMagnetometerCollector: AccelerometerAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerAndMagnetometerCollector")
        requireNotNull(accelerometerAndMagnetometerCollector)

        val measurement = AccelerometerAndMagnetometerSyncedSensorMeasurement()
        val fusedAttitude1 = getAttitude()

        val accelerometerProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        every { accelerometerProcessorSpy.process(measurement) }.returns(true)
        every { accelerometerProcessorSpy.fusedAttitude }.returns(fusedAttitude1)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val listener = accelerometerAndMagnetometerCollector.measurementListener
        requireNotNull(listener)
        listener.onMeasurement(accelerometerAndMagnetometerCollector, measurement)

        val fusedAttitude2: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude2)
        assertEquals(fusedAttitude1, fusedAttitude2)
    }

    @Test
    fun accelerometerCollector_whenSyncedMeasurementProcessedEstimateCoordinateTransformationAndEulerAnglesDisableAndListener_updatesFusedAttitudeAndNotifies() {
        val estimator = GeomagneticAttitudeEstimator(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val accelerometerAndMagnetometerCollector: AccelerometerAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerAndMagnetometerCollector")
        requireNotNull(accelerometerAndMagnetometerCollector)

        val timestamp = System.nanoTime()
        val measurement = AccelerometerAndMagnetometerSyncedSensorMeasurement(timestamp = timestamp)
        val fusedAttitude1 = getAttitude()

        val accelerometerProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        every { accelerometerProcessorSpy.process(measurement) }.returns(true)
        every { accelerometerProcessorSpy.fusedAttitude }.returns(fusedAttitude1)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val listener = accelerometerAndMagnetometerCollector.measurementListener
        requireNotNull(listener)
        listener.onMeasurement(accelerometerAndMagnetometerCollector, measurement)

        val fusedAttitude2: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude2)
        assertEquals(fusedAttitude1, fusedAttitude2)

        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                fusedAttitude2,
                timestamp,
                null,
                null,
                null,
                null
            )
        }
    }

    @Test
    fun accelerometerSyncer_whenSyncedMeasurementProcessedEstimateCoordinateTransformationAndEulerAnglesEnableAndListener_updatesFusedAttitudeAndNotifies() {
        val estimator = GeomagneticAttitudeEstimator(
            context,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = true,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val accelerometerAndMagnetometerCollector: AccelerometerAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerAndMagnetometerCollector")
        requireNotNull(accelerometerAndMagnetometerCollector)

        val timestamp = System.nanoTime()
        val measurement = AccelerometerAndMagnetometerSyncedSensorMeasurement(timestamp = timestamp)
        val fusedAttitude1 = getAttitude()

        val accelerometerProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        every { accelerometerProcessorSpy.process(measurement) }.returns(true)
        every { accelerometerProcessorSpy.fusedAttitude }.returns(fusedAttitude1)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val listener = accelerometerAndMagnetometerCollector.measurementListener
        requireNotNull(listener)
        listener.onMeasurement(accelerometerAndMagnetometerCollector, measurement)

        val fusedAttitude2: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude2)
        assertEquals(fusedAttitude1, fusedAttitude2)

        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        requireNotNull(coordinateTransformation)
        assertEquals(
            CoordinateTransformation(
                fusedAttitude1,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            ), coordinateTransformation
        )

        val eulerAngles1: DoubleArray? = estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles1)
        val eulerAngles2 = fusedAttitude1.toEulerAngles()
        assertArrayEquals(eulerAngles1, eulerAngles2, 0.0)

        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                fusedAttitude2,
                timestamp,
                eulerAngles2[0],
                eulerAngles2[1],
                eulerAngles2[2],
                coordinateTransformation
            )
        }
    }

    private fun getLocation(): Location {
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES)
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

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

        const val MIN_ANGLE_DEGREES = -45.0
        const val MAX_ANGLE_DEGREES = 45.0

        fun getAttitude(): Quaternion {
            val randomizer = UniformRandomizer()
            val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))

            return Quaternion(roll, pitch, yaw)
        }
    }
}