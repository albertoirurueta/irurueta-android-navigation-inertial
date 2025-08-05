package com.irurueta.android.navigation.inertial.estimators.attitude

import android.content.Context
import android.location.Location
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MeanAveragingFilter
import com.irurueta.android.navigation.inertial.processors.attitude.AccelerometerGeomagneticAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.attitude.GeomagneticAttitudeProcessor
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.statistics.UniformRandomizer
import io.mockk.*
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import org.junit.After
import org.junit.Assert.*
import org.junit.Ignore
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import java.util.*

@Ignore("possible memory leak")
@RunWith(RobolectricTestRunner::class)
class GeomagneticAttitudeEstimator2Test {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var attitudeAvailableListener:
            GeomagneticAttitudeEstimator2.OnAttitudeAvailableListener

    @MockK(relaxUnitFun = true)
    private lateinit var accuracyChangedListener:
            GeomagneticAttitudeEstimator2.OnAccuracyChangedListener

    @MockK(relaxUnitFun = true)
    private lateinit var bufferFilledListener: GeomagneticAttitudeEstimator2.OnBufferFilledListener

    @MockK
    private lateinit var location: Location

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context)

        // check
        assertSame(context, estimator.context)
        assertNull(estimator.location)
        assertTrue(estimator.adjustGravityNorm)
        assertEquals(SensorDelay.GAME, estimator.sensorDelay)
        assertTrue(estimator.useAccelerometer)
        assertTrue(estimator.startOffsetEnabled)
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
        assertNull(estimator.timestamp)
        assertFalse(estimator.useWorldMagneticModel)
        assertFalse(estimator.useAccurateLevelingEstimator)
        assertFalse(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateEulerAngles)
        assertNull(estimator.attitudeAvailableListener)
        assertNull(estimator.accuracyChangedListener)
        assertNull(estimator.bufferFilledListener)
    }

    @Test
    fun constructor_whenAllProperties_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val worldMagneticModel = WorldMagneticModel()
        val timestamp = Date()
        val accelerometerAveragingFilter = MeanAveragingFilter()
        val estimator = GeomagneticAttitudeEstimator2(
            context,
            location,
            SensorDelay.NORMAL,
            useAccelerometer = false,
            startOffsetEnabled = false,
            AccelerometerSensorType.ACCELEROMETER,
            MagnetometerSensorType.MAGNETOMETER,
            accelerometerAveragingFilter,
            worldMagneticModel,
            timestamp,
            useWorldMagneticModel = true,
            useAccurateLevelingEstimator = true,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeAvailableListener,
            accuracyChangedListener = accuracyChangedListener,
            bufferFilledListener = bufferFilledListener
        )

        // check
        assertSame(context, estimator.context)
        assertSame(location, estimator.location)
        assertTrue(estimator.adjustGravityNorm)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertFalse(estimator.useAccelerometer)
        assertFalse(estimator.startOffsetEnabled)
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
        assertTrue(estimator.useAccurateLevelingEstimator)
        assertTrue(estimator.estimateCoordinateTransformation)
        assertFalse(estimator.estimateEulerAngles)
        assertSame(attitudeAvailableListener, estimator.attitudeAvailableListener)
        assertSame(accuracyChangedListener, estimator.accuracyChangedListener)
        assertSame(bufferFilledListener, estimator.bufferFilledListener)
    }

    @Test(expected = IllegalStateException::class)
    fun constructor_whenAccurateLevelingEnabledAndMissingLocation_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        GeomagneticAttitudeEstimator2(context, useAccurateLevelingEstimator = true)
    }

    @Test
    fun timestamp_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context)

        // check default value
        assertNull(estimator.attitudeAvailableListener)

        // set new value
        estimator.attitudeAvailableListener = attitudeAvailableListener

        // check
        assertSame(attitudeAvailableListener, estimator.attitudeAvailableListener)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context)

        // check default value
        assertNull(estimator.accuracyChangedListener)

        // set new value
        estimator.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, estimator.accuracyChangedListener)
    }

    @Test
    fun bufferFilledListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context)

        // check default value
        assertNull(estimator.bufferFilledListener)

        // set new value
        estimator.bufferFilledListener = bufferFilledListener

        // check
        assertSame(bufferFilledListener, estimator.bufferFilledListener)
    }

    @Test
    fun location_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context)

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

    @Test(expected = IllegalStateException::class)
    fun location_whenRunningAndNull_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = GeomagneticAttitudeEstimator2(context, location)

        assertFalse(estimator.running)
        assertSame(location, estimator.location)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        // set new value
        estimator.location = null
    }

    @Test
    fun adjustGravityNorm_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context)

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
    fun useAccurateLevelingEstimator_whenTrueValueNotRunningAndLocationAvailable_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = GeomagneticAttitudeEstimator2(context, location)

        val accelerometerProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: GeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        // check default value
        assertFalse(estimator.useAccurateLevelingEstimator)
        assertFalse(estimator.running)
        assertSame(location, estimator.location)

        // set new value
        estimator.useAccurateLevelingEstimator = true

        // check
        assertTrue(estimator.useAccurateLevelingEstimator)
        assertTrue(accelerometerProcessor.useAccurateLevelingProcessor)
        assertTrue(gravityProcessor.useAccurateLevelingProcessor)
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateLevelingEstimator_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        estimator.useAccurateLevelingEstimator = true
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateLevelingEstimator_whenNotRunningAndNoLocation_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context)

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
        val estimator = GeomagneticAttitudeEstimator2(context)

        val accelerometerProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: GeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        // check default values
        assertFalse(estimator.running)
        assertNull(estimator.location)
        assertFalse(estimator.useAccurateLevelingEstimator)

        // set new value
        estimator.useAccurateLevelingEstimator = false

        // check
        assertFalse(estimator.useAccurateLevelingEstimator)
        assertFalse(accelerometerProcessor.useAccurateLevelingProcessor)
        assertFalse(gravityProcessor.useAccurateLevelingProcessor)
    }

    @Test
    fun worldMagneticModel_whenNotRunningAndNotUseWorldMagneticModel_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context, useWorldMagneticModel = true)

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

    @Test(expected = IllegalStateException::class)
    fun worldMagneticModel_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context)

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
        val estimator = GeomagneticAttitudeEstimator2(context)

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

    @Test(expected = IllegalStateException::class)
    fun useWorldMagneticModel_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        // set new value
        estimator.useWorldMagneticModel = true
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenRunningAndUseAccelerometer_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        estimator.start()
    }

    @Test
    fun start_whenNotRunningAndUseAccelerometer_resetsAndStartsSyncer() {
        val timestamp = System.nanoTime()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context, useAccelerometer = true)

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

        val accelerometerAndMagnetometerSyncer: AccelerometerAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndMagnetometerSyncer")
        requireNotNull(accelerometerAndMagnetometerSyncer)
        val accelerometerAndMagnetometerSyncerSpy = spyk(accelerometerAndMagnetometerSyncer)
        every { accelerometerAndMagnetometerSyncerSpy.start(timestamp) }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerAndMagnetometerSyncer",
            accelerometerAndMagnetometerSyncerSpy
        )

        val gravityAndMagnetometerSyncer: GravityAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndMagnetometerSyncer")
        requireNotNull(gravityAndMagnetometerSyncer)
        val gravityAndMagnetometerSyncerSpy = spyk(gravityAndMagnetometerSyncer)
        estimator.setPrivateProperty(
            "gravityAndMagnetometerSyncer",
            gravityAndMagnetometerSyncerSpy
        )

        assertFalse(estimator.running)

        assertTrue(estimator.start(timestamp))

        verify(exactly = 1) { accelerometerProcessorSpy.reset() }
        verify(exactly = 1) { accelerometerAndMagnetometerSyncerSpy.start(timestamp) }
        verify { gravityProcessorSpy wasNot Called }
        verify { gravityAndMagnetometerSyncerSpy wasNot Called }
    }

    @Test
    fun start_whenNotRunningAndAccelerometerNotUsed_resetsAndStartsSyncer() {
        val timestamp = System.nanoTime()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context, useAccelerometer = false)

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

        val accelerometerAndMagnetometerSyncer: AccelerometerAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndMagnetometerSyncer")
        requireNotNull(accelerometerAndMagnetometerSyncer)
        val accelerometerAndMagnetometerSyncerSpy = spyk(accelerometerAndMagnetometerSyncer)
        estimator.setPrivateProperty(
            "accelerometerAndMagnetometerSyncer",
            accelerometerAndMagnetometerSyncerSpy
        )

        val gravityAndMagnetometerSyncer: GravityAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndMagnetometerSyncer")
        requireNotNull(gravityAndMagnetometerSyncer)
        val gravityAndMagnetometerSyncerSpy = spyk(gravityAndMagnetometerSyncer)
        every { gravityAndMagnetometerSyncerSpy.start(timestamp) }.returns(true)
        estimator.setPrivateProperty(
            "gravityAndMagnetometerSyncer",
            gravityAndMagnetometerSyncerSpy
        )

        assertFalse(estimator.running)

        assertTrue(estimator.start(timestamp))

        verify(exactly = 1) { gravityProcessorSpy.reset() }
        verify(exactly = 1) { gravityAndMagnetometerSyncerSpy.start(timestamp) }
        verify { accelerometerProcessorSpy wasNot Called }
        verify { accelerometerAndMagnetometerSyncerSpy wasNot Called }
    }

    @Test
    fun stop_stopsInternalSyncers() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context)

        // setup spies
        val accelerometerAndMagnetometerSyncer: AccelerometerAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndMagnetometerSyncer")
        requireNotNull(accelerometerAndMagnetometerSyncer)
        val accelerometerAndMagnetometerSyncerSpy = spyk(accelerometerAndMagnetometerSyncer)
        estimator.setPrivateProperty(
            "accelerometerAndMagnetometerSyncer",
            accelerometerAndMagnetometerSyncerSpy
        )

        val gravityAndMagnetometerSyncer: GravityAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndMagnetometerSyncer")
        requireNotNull(gravityAndMagnetometerSyncer)
        val gravityAndMagnetometerSyncerSpy = spyk(gravityAndMagnetometerSyncer)
        estimator.setPrivateProperty(
            "gravityAndMagnetometerSyncer",
            gravityAndMagnetometerSyncerSpy
        )

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        // stop
        estimator.stop()

        // check
        assertFalse(estimator.running)
        verify(exactly = 1) { accelerometerAndMagnetometerSyncerSpy.stop() }
        verify(exactly = 1) { gravityAndMagnetometerSyncerSpy.stop() }
    }

    @Test
    fun gravitySyncer_whenAccuracyChangedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context)

        val gravityAndMagnetometerSyncer: GravityAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndMagnetometerSyncer")
        requireNotNull(gravityAndMagnetometerSyncer)

        val listener = gravityAndMagnetometerSyncer.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            gravityAndMagnetometerSyncer,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )
    }

    @Test
    fun gravitySyncer_whenAccuracyChangedAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val gravityAndMagnetometerSyncer: GravityAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndMagnetometerSyncer")
        requireNotNull(gravityAndMagnetometerSyncer)

        val listener = gravityAndMagnetometerSyncer.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            gravityAndMagnetometerSyncer,
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
    fun gravitySyncer_whenBufferFilledAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context)

        val gravityAndMagnetometerSyncer: GravityAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndMagnetometerSyncer")
        requireNotNull(gravityAndMagnetometerSyncer)

        val listener = gravityAndMagnetometerSyncer.bufferFilledListener
        requireNotNull(listener)
        listener.onBufferFilled(
            gravityAndMagnetometerSyncer,
            SensorType.GRAVITY
        )
    }

    @Test
    fun gravitySyncer_whenBufferFilledAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(
            context,
            bufferFilledListener = bufferFilledListener
        )

        val gravityAndMagnetometerSyncer: GravityAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndMagnetometerSyncer")
        requireNotNull(gravityAndMagnetometerSyncer)

        val listener = gravityAndMagnetometerSyncer.bufferFilledListener
        requireNotNull(listener)
        listener.onBufferFilled(
            gravityAndMagnetometerSyncer,
            SensorType.GRAVITY
        )

        // check
        verify(exactly = 1) {
            bufferFilledListener.onBufferFilled(
                estimator,
                SensorType.GRAVITY
            )
        }
    }

    @Test
    fun gravitySyncer_whenSyncedMeasurementAndNotProcessed_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(
            context,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val gravityAndMagnetometerSyncer: GravityAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndMagnetometerSyncer")
        requireNotNull(gravityAndMagnetometerSyncer)

        val measurement = GravityAndMagnetometerSyncedSensorMeasurement()

        val gravityProcessor: GeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(measurement) }.returns(false)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val listener = gravityAndMagnetometerSyncer.syncedMeasurementListener
        requireNotNull(listener)
        listener.onSyncedMeasurements(gravityAndMagnetometerSyncer, measurement)

        verify { attitudeAvailableListener wasNot Called }
    }

    @Test
    fun gravitySyncer_whenSyncedMeasurementProcessedAndNoListener_updatesFusedAttitude() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context)

        val gravityAndMagnetometerSyncer: GravityAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndMagnetometerSyncer")
        requireNotNull(gravityAndMagnetometerSyncer)

        val measurement = GravityAndMagnetometerSyncedSensorMeasurement()
        val fusedAttitude1 = getAttitude()

        val gravityProcessor: GeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(measurement) }.returns(true)
        every { gravityProcessorSpy.fusedAttitude }.returns(fusedAttitude1)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val listener = gravityAndMagnetometerSyncer.syncedMeasurementListener
        requireNotNull(listener)
        listener.onSyncedMeasurements(gravityAndMagnetometerSyncer, measurement)

        val fusedAttitude2: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude2)
        assertEquals(fusedAttitude1, fusedAttitude2)
    }

    @Test
    fun gravitySyncer_whenSyncedMeasurementProcessedEstimateCoordinateTransformationAndEulerAnglesDisableAndListener_updatesFusedAttitudeAndNotifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val gravityAndMagnetometerSyncer: GravityAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndMagnetometerSyncer")
        requireNotNull(gravityAndMagnetometerSyncer)

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

        val listener = gravityAndMagnetometerSyncer.syncedMeasurementListener
        requireNotNull(listener)
        listener.onSyncedMeasurements(gravityAndMagnetometerSyncer, measurement)

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
    fun gravitySyncer_whenSyncedMeasurementProcessedEstimateCoordinateTransformationAndEulerAnglesEnableAndListener_updatesFusedAttitudeAndNotifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(
            context,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = true,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val gravityAndMagnetometerSyncer: GravityAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndMagnetometerSyncer")
        requireNotNull(gravityAndMagnetometerSyncer)

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

        val listener = gravityAndMagnetometerSyncer.syncedMeasurementListener
        requireNotNull(listener)
        listener.onSyncedMeasurements(gravityAndMagnetometerSyncer, measurement)

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
    fun accelerometerSyncer_whenAccuracyChangedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context)

        val accelerometerAndMagnetometerSyncer: AccelerometerAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndMagnetometerSyncer")
        requireNotNull(accelerometerAndMagnetometerSyncer)

        val listener = accelerometerAndMagnetometerSyncer.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            accelerometerAndMagnetometerSyncer,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )
    }

    @Test
    fun accelerometerSyncer_whenAccuracyChangedAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val accelerometerAndMagnetometerSyncer: AccelerometerAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndMagnetometerSyncer")
        requireNotNull(accelerometerAndMagnetometerSyncer)

        val listener = accelerometerAndMagnetometerSyncer.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            accelerometerAndMagnetometerSyncer,
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
    fun accelerometerSyncer_whenBufferFilledAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context)

        val accelerometerAndMagnetometerSyncer: AccelerometerAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndMagnetometerSyncer")
        requireNotNull(accelerometerAndMagnetometerSyncer)

        val listener = accelerometerAndMagnetometerSyncer.bufferFilledListener
        requireNotNull(listener)
        listener.onBufferFilled(
            accelerometerAndMagnetometerSyncer,
            SensorType.GRAVITY
        )
    }

    @Test
    fun accelerometerSyncer_whenBufferFilledAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(
            context,
            bufferFilledListener = bufferFilledListener
        )

        val accelerometerAndMagnetometerSyncer: AccelerometerAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndMagnetometerSyncer")
        requireNotNull(accelerometerAndMagnetometerSyncer)

        val listener = accelerometerAndMagnetometerSyncer.bufferFilledListener
        requireNotNull(listener)
        listener.onBufferFilled(
            accelerometerAndMagnetometerSyncer,
            SensorType.GRAVITY
        )

        // check
        verify(exactly = 1) {
            bufferFilledListener.onBufferFilled(
                estimator,
                SensorType.GRAVITY
            )
        }
    }

    @Test
    fun accelerometerSyncer_whenSyncedMeasurementAndNotProcessed_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(
            context,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val accelerometerAndMagnetometerSyncer: AccelerometerAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndMagnetometerSyncer")
        requireNotNull(accelerometerAndMagnetometerSyncer)

        val measurement = AccelerometerAndMagnetometerSyncedSensorMeasurement()

        val accelerometerProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        every { accelerometerProcessorSpy.process(measurement) }.returns(false)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val listener = accelerometerAndMagnetometerSyncer.syncedMeasurementListener
        requireNotNull(listener)
        listener.onSyncedMeasurements(accelerometerAndMagnetometerSyncer, measurement)

        verify { attitudeAvailableListener wasNot Called }
    }

    @Test
    fun accelerometerSyncer_whenSyncedMeasurementProcessedAndNoListener_updatesFusedAttitude() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(context)

        val accelerometerAndMagnetometerSyncer: AccelerometerAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndMagnetometerSyncer")
        requireNotNull(accelerometerAndMagnetometerSyncer)

        val measurement = AccelerometerAndMagnetometerSyncedSensorMeasurement()
        val fusedAttitude1 = getAttitude()

        val accelerometerProcessor: AccelerometerGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        every { accelerometerProcessorSpy.process(measurement) }.returns(true)
        every { accelerometerProcessorSpy.fusedAttitude }.returns(fusedAttitude1)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val listener = accelerometerAndMagnetometerSyncer.syncedMeasurementListener
        requireNotNull(listener)
        listener.onSyncedMeasurements(accelerometerAndMagnetometerSyncer, measurement)

        val fusedAttitude2: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude2)
        assertEquals(fusedAttitude1, fusedAttitude2)
    }

    @Test
    fun accelerometerSyncer_whenSyncedMeasurementProcessedEstimateCoordinateTransformationAndEulerAnglesDisableAndListener_updatesFusedAttitudeAndNotifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val accelerometerAndMagnetometerSyncer: AccelerometerAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndMagnetometerSyncer")
        requireNotNull(accelerometerAndMagnetometerSyncer)

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

        val listener = accelerometerAndMagnetometerSyncer.syncedMeasurementListener
        requireNotNull(listener)
        listener.onSyncedMeasurements(accelerometerAndMagnetometerSyncer, measurement)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GeomagneticAttitudeEstimator2(
            context,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = true,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val accelerometerAndMagnetometerSyncer: AccelerometerAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndMagnetometerSyncer")
        requireNotNull(accelerometerAndMagnetometerSyncer)

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

        val listener = accelerometerAndMagnetometerSyncer.syncedMeasurementListener
        requireNotNull(listener)
        listener.onSyncedMeasurements(accelerometerAndMagnetometerSyncer, measurement)

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