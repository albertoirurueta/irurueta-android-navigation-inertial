package com.irurueta.android.navigation.inertial.estimators.attitude

import android.content.Context
import android.location.Location
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MedianAveragingFilter
import com.irurueta.android.navigation.inertial.processors.attitude.*
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.statistics.UniformRandomizer
//import io.mockk.*
//import io.mockk.impl.annotations.MockK
//import io.mockk.junit4.MockKRule
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
import org.mockito.kotlin.doReturn
import org.mockito.kotlin.never
import org.mockito.kotlin.only
import org.mockito.kotlin.spy
import org.mockito.kotlin.times
import org.mockito.kotlin.verify
import org.mockito.kotlin.verifyNoInteractions
import org.mockito.kotlin.whenever
import org.robolectric.RobolectricTestRunner
import java.util.*

//@Ignore("Possible memory leak when running this test")
@RunWith(RobolectricTestRunner::class)
class DoubleFusedGeomagneticAttitudeEstimator2Test {

    @get:Rule
    val mockitoRule: MockitoRule = MockitoJUnit.rule()

//    @get:Rule
//    val mockkRule = MockKRule(this)

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var attitudeListener:
            DoubleFusedGeomagneticAttitudeEstimator2.OnAttitudeAvailableListener

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var accuracyChangedListener:
            DoubleFusedGeomagneticAttitudeEstimator2.OnAccuracyChangedListener

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var bufferFilledListener:
            DoubleFusedGeomagneticAttitudeEstimator2.OnBufferFilledListener

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
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

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
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            estimator.gyroscopeSensorType
        )
        assertNull(estimator.worldMagneticModel)
        assertNotNull(estimator.timestamp)
        assertFalse(estimator.useWorldMagneticModel)
        assertFalse(estimator.useAccurateLevelingEstimator)
        assertFalse(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertFalse(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateEulerAngles)
        assertNull(estimator.attitudeAvailableListener)
        assertNull(estimator.accuracyChangedListener)
        assertNull(estimator.bufferFilledListener)
        assertTrue(estimator.useIndirectInterpolation)
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            estimator.interpolationValue,
            0.0
        )
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.indirectInterpolationWeight,
            0.0
        )
        assertEquals(0.0, estimator.gyroscopeTimeIntervalSeconds, 0.0)
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            estimator.outlierThreshold,
            0.0
        )
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.outlierPanicThreshold,
            0.0
        )
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.panicCounterThreshold
        )
        assertFalse(estimator.running)
    }

    @Test
    fun constructor_whenAllProperties_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val accelerometerAveragingFilter = MedianAveragingFilter()
        val worldMagneticModel = WorldMagneticModel()
        val timestamp = Date()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(
            context,
            location,
            SensorDelay.NORMAL,
            useAccelerometer = false,
            startOffsetEnabled = false,
            AccelerometerSensorType.ACCELEROMETER,
            MagnetometerSensorType.MAGNETOMETER,
            accelerometerAveragingFilter,
            GyroscopeSensorType.GYROSCOPE,
            worldMagneticModel,
            timestamp,
            useWorldMagneticModel = true,
            useAccurateLevelingEstimator = true,
            useAccurateRelativeGyroscopeAttitudeEstimator = true,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeListener,
            accuracyChangedListener = accuracyChangedListener,
            bufferFilledListener = bufferFilledListener,
            adjustGravityNorm = false
        )

        // check
        assertSame(context, estimator.context)
        assertSame(location, estimator.location)
        assertFalse(estimator.adjustGravityNorm)
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
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            estimator.gyroscopeSensorType
        )
        assertSame(worldMagneticModel, estimator.worldMagneticModel)
        assertEquals(timestamp, estimator.timestamp)
        assertTrue(estimator.useWorldMagneticModel)
        assertTrue(estimator.useAccurateLevelingEstimator)
        assertTrue(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertTrue(estimator.estimateCoordinateTransformation)
        assertFalse(estimator.estimateEulerAngles)
        assertSame(attitudeListener, estimator.attitudeAvailableListener)
        assertSame(accuracyChangedListener, estimator.accuracyChangedListener)
        assertSame(bufferFilledListener, estimator.bufferFilledListener)
        assertTrue(estimator.useIndirectInterpolation)
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            estimator.interpolationValue,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.indirectInterpolationWeight,
            0.0
        )
        assertEquals(0.0, estimator.gyroscopeTimeIntervalSeconds, 0.0)
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            estimator.outlierThreshold,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.outlierPanicThreshold,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.panicCounterThreshold
        )
        assertFalse(estimator.running)
    }

    @Test
    fun location_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

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
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

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
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context, location)

        assertFalse(estimator.running)
        assertSame(location, estimator.location)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        // set new value
        estimator.location = null
    }

    @Test
    fun adjustGravityNorm_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator2(context)

        // check default value
        assertTrue(estimator.adjustGravityNorm)

        val gravityProcessor: GravityProcessor? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "gravityProcessor")
        requireNotNull(gravityProcessor)
        assertTrue(gravityProcessor.adjustGravityNorm)

        val accelerometerGravityProcessor: AccelerometerGravityProcessor? =
            getPrivateProperty(
                BaseLevelingEstimator2::class,
                estimator,
                "accelerometerGravityProcessor"
            )
        requireNotNull(accelerometerGravityProcessor)
        assertTrue(accelerometerGravityProcessor.adjustGravityNorm)

        // set new value
        estimator.adjustGravityNorm = false

        // check
        assertFalse(estimator.adjustGravityNorm)
        assertFalse(gravityProcessor.adjustGravityNorm)
        assertFalse(accelerometerGravityProcessor.adjustGravityNorm)
    }

    @Test
    fun worldMagneticModel_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        // check default value
        assertFalse(estimator.running)
        assertNull(estimator.worldMagneticModel)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        assertNull(accelerometerProcessor.worldMagneticModel)
        assertNull(gravityProcessor.worldMagneticModel)

        val accelerometerProcessorSpy = spy(accelerometerProcessor)
//        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        // set new value
        val worldMagneticModel = WorldMagneticModel()
        estimator.worldMagneticModel = worldMagneticModel

        // check
        assertSame(worldMagneticModel, estimator.worldMagneticModel)

        verify(accelerometerProcessorSpy, times(1)).worldMagneticModel = worldMagneticModel
//        verify { accelerometerProcessorSpy.worldMagneticModel = worldMagneticModel }
        verify(gravityProcessorSpy, times(1)).worldMagneticModel = worldMagneticModel
//        verify { gravityProcessorSpy.worldMagneticModel = worldMagneticModel }

        assertSame(worldMagneticModel, accelerometerProcessor.worldMagneticModel)
        assertSame(worldMagneticModel, gravityProcessor.worldMagneticModel)
    }

    @Test(expected = IllegalStateException::class)
    fun worldMagneticModel_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        // set as running
        estimator.setPrivateProperty("running", true)

        estimator.worldMagneticModel = WorldMagneticModel()
    }

    @Test
    fun timestamp_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        // check default value
        assertNotNull(estimator.timestamp)

        // set new value
        val timestamp = Date()
        estimator.timestamp = timestamp

        // check
        assertSame(timestamp, estimator.timestamp)
    }

    @Test
    fun attitudeAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        // check default value
        assertNull(estimator.attitudeAvailableListener)

        // set new value
        estimator.attitudeAvailableListener = attitudeListener

        // check
        assertSame(attitudeListener, estimator.attitudeAvailableListener)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

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
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        // check default value
        assertNull(estimator.bufferFilledListener)

        // set new value
        estimator.bufferFilledListener = bufferFilledListener

        // check
        assertSame(bufferFilledListener, estimator.bufferFilledListener)
    }

    @Test
    fun useWorldMagneticModel_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        // check
        assertFalse(estimator.running)
        assertFalse(estimator.useWorldMagneticModel)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        assertFalse(accelerometerProcessor.useWorldMagneticModel)
        assertFalse(gravityProcessor.useWorldMagneticModel)

        val accelerometerProcessorSpy = spy(accelerometerProcessor)
//        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        // set new value
        estimator.useWorldMagneticModel = true

        // check
        assertTrue(estimator.useWorldMagneticModel)

        verify(accelerometerProcessorSpy, times(1)).useWorldMagneticModel = true
//        verify(exactly = 1) { accelerometerProcessorSpy.useWorldMagneticModel = true }
        verify(gravityProcessorSpy, times(1)).useWorldMagneticModel = true
//        verify(exactly = 1) { gravityProcessorSpy.useWorldMagneticModel = true }

        assertTrue(accelerometerProcessor.useWorldMagneticModel)
        assertTrue(gravityProcessor.useWorldMagneticModel)
    }

    @Test(expected = IllegalStateException::class)
    fun useWorldMagneticModel_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        estimator.useWorldMagneticModel = true
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateLevelingEstimator_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        assertFalse(estimator.useAccurateLevelingEstimator)
        assertFalse(estimator.running)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        estimator.useAccurateLevelingEstimator = true
    }

    @Test
    fun useAccurateLevelingEstimator_whenNotRunningNoLocationAndSetToFalse_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        assertFalse(estimator.useAccurateLevelingEstimator)
        assertNull(estimator.location)
        assertFalse(estimator.running)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spy(accelerometerProcessor)
//        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        estimator.useAccurateLevelingEstimator = false

        // check
        assertFalse(estimator.useAccurateLevelingEstimator)
        assertFalse(accelerometerProcessor.useAccurateLevelingProcessor)
        assertFalse(gravityProcessor.useAccurateLevelingProcessor)

        verify(accelerometerProcessorSpy, times(1)).useAccurateLevelingProcessor = false
//        verify(exactly = 1) { accelerometerProcessorSpy.useAccurateLevelingProcessor = false }
        verify(gravityProcessorSpy, times(1)).useAccurateLevelingProcessor = false
//        verify(exactly = 1) { gravityProcessorSpy.useAccurateLevelingProcessor = false }
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateLevelingEstimator_whenNotRunningNoLocationAndSetToTrue_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        assertFalse(estimator.useAccurateLevelingEstimator)
        assertNull(estimator.location)
        assertFalse(estimator.running)

        estimator.useAccurateLevelingEstimator = true
    }

    @Test
    fun useAccurateLevelingEstimator_whenNotRunningAndLocationAndSetToTrue_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context, location)

        // check default value
        assertFalse(estimator.running)
        assertSame(location, estimator.location)
        assertFalse(estimator.useAccurateLevelingEstimator)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spy(accelerometerProcessor)
//        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        // set new value
        estimator.useAccurateLevelingEstimator = true

        // check
        assertTrue(estimator.useAccurateLevelingEstimator)
        assertTrue(accelerometerProcessor.useAccurateLevelingProcessor)
        assertTrue(gravityProcessor.useAccurateLevelingProcessor)

        verify(accelerometerProcessorSpy, times(1)).useAccurateLevelingProcessor = true
//        verify(exactly = 1) { accelerometerProcessorSpy.useAccurateLevelingProcessor = true }
        verify(gravityProcessorSpy, times(1)).useAccurateLevelingProcessor = true
//        verify(exactly = 1) { gravityProcessorSpy.useAccurateLevelingProcessor = true }
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateRelativeGyroscopeAttitudeEstimator_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        assertFalse(estimator.running)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        estimator.useAccurateRelativeGyroscopeAttitudeEstimator = true
    }

    @Test
    fun useAccurateRelativeGyroscopeAttitudeEstimator_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        assertFalse(estimator.running)
        assertFalse(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spy(accelerometerProcessor)
//        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        // set new value
        estimator.useAccurateRelativeGyroscopeAttitudeEstimator = true

        // check
        assertTrue(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertTrue(accelerometerProcessorSpy.useAccurateRelativeGyroscopeAttitudeProcessor)
        assertTrue(gravityProcessorSpy.useAccurateRelativeGyroscopeAttitudeProcessor)

        verify(accelerometerProcessorSpy, times(1)).useAccurateRelativeGyroscopeAttitudeProcessor = true
/*        verify(exactly = 1) {
            accelerometerProcessorSpy.useAccurateRelativeGyroscopeAttitudeProcessor = true
        }*/
        verify(gravityProcessorSpy, times(1)).useAccurateRelativeGyroscopeAttitudeProcessor = true
/*        verify(exactly = 1) {
            gravityProcessorSpy.useAccurateRelativeGyroscopeAttitudeProcessor = true
        }*/
    }

    @Test
    fun useIndirectInterpolation_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        // check default value
        assertTrue(estimator.useIndirectInterpolation)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spy(accelerometerProcessor)
//        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        // set new value
        estimator.useIndirectInterpolation = false

        // check
        assertFalse(estimator.useIndirectInterpolation)
        assertFalse(accelerometerProcessorSpy.useIndirectInterpolation)
        assertFalse(gravityProcessorSpy.useIndirectInterpolation)

        verify(accelerometerProcessorSpy, times(1)).useIndirectInterpolation = false
//        verify(exactly = 1) { accelerometerProcessorSpy.useIndirectInterpolation = false }
        verify(gravityProcessorSpy, times(1)).useIndirectInterpolation = false
//        verify(exactly = 1) { gravityProcessorSpy.useIndirectInterpolation = false }
    }

    @Test
    fun interpolationValue_whenOutOfRange_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            estimator.interpolationValue,
            0.0
        )

        assertThrows(IllegalArgumentException::class.java) {
            estimator.interpolationValue = -1.0
        }
        assertThrows(IllegalArgumentException::class.java) {
            estimator.interpolationValue = 2.0
        }
    }

    @Test
    fun interpolationValue_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            estimator.interpolationValue,
            0.0
        )

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spy(accelerometerProcessor)
//        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        estimator.interpolationValue = value

        // check
        assertEquals(value, estimator.interpolationValue, 0.0)
        assertEquals(value, accelerometerProcessorSpy.interpolationValue, 0.0)
        assertEquals(value, gravityProcessorSpy.interpolationValue, 0.0)

        verify(accelerometerProcessorSpy, times(1)).interpolationValue = value
//        verify(exactly = 1) { accelerometerProcessorSpy.interpolationValue = value }
        verify(gravityProcessorSpy, times(1)).interpolationValue = value
//        verify(exactly = 1) { gravityProcessorSpy.interpolationValue = value }
    }

    @Test(expected = IllegalArgumentException::class)
    fun indirectInterpolationWeight_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        estimator.indirectInterpolationWeight = 0.0
    }

    @Test
    fun indirectInterpolationWeight_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        // check default value
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.indirectInterpolationWeight,
            0.0
        )

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spy(accelerometerProcessor)
//        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        // sets new value
        val randomizer = UniformRandomizer()
        val indirectInterpolationWeight = randomizer.nextDouble()
        estimator.indirectInterpolationWeight = indirectInterpolationWeight

        // check
        assertEquals(indirectInterpolationWeight, estimator.indirectInterpolationWeight, 0.0)
        assertEquals(
            indirectInterpolationWeight,
            accelerometerProcessorSpy.indirectInterpolationWeight,
            0.0
        )
        assertEquals(
            indirectInterpolationWeight,
            gravityProcessorSpy.indirectInterpolationWeight,
            0.0
        )

        verify(accelerometerProcessorSpy, times(1)).indirectInterpolationWeight = indirectInterpolationWeight
/*        verify(exactly = 1) {
            accelerometerProcessorSpy.indirectInterpolationWeight = indirectInterpolationWeight
        }*/
        verify(gravityProcessorSpy, times(1)).indirectInterpolationWeight = indirectInterpolationWeight
/*        verify(exactly = 1) {
            gravityProcessorSpy.indirectInterpolationWeight = indirectInterpolationWeight
        }*/
    }

    @Test
    fun gyroscopeTimeIntervalSeconds_whenUseAccelerometer_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context, useAccelerometer = true)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spy(accelerometerProcessor)
//        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        doReturn(TIME_INTERVAL).whenever(accelerometerProcessorSpy).gyroscopeTimeIntervalSeconds
//        every { accelerometerProcessorSpy.gyroscopeTimeIntervalSeconds }.returns(TIME_INTERVAL)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        assertEquals(TIME_INTERVAL, estimator.gyroscopeTimeIntervalSeconds, 0.0)

        verify(accelerometerProcessorSpy, only()).gyroscopeTimeIntervalSeconds
//        verify(exactly = 1) { accelerometerProcessorSpy.gyroscopeTimeIntervalSeconds }
        verifyNoInteractions(gravityProcessorSpy)
//        verify { gravityProcessorSpy wasNot Called }
    }

    @Test
    fun gyroscopeTimeIntervalSeconds_whenNotUseAccelerometer_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context, useAccelerometer = false)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spy(accelerometerProcessor)
//        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        doReturn(TIME_INTERVAL).whenever(gravityProcessorSpy).gyroscopeTimeIntervalSeconds
//        every { gravityProcessorSpy.gyroscopeTimeIntervalSeconds }.returns(TIME_INTERVAL)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        assertEquals(TIME_INTERVAL, estimator.gyroscopeTimeIntervalSeconds, 0.0)

        verify(gravityProcessorSpy, only()).gyroscopeTimeIntervalSeconds
//        verify(exactly = 1) { gravityProcessorSpy.gyroscopeTimeIntervalSeconds }
        verifyNoInteractions(accelerometerProcessorSpy)
//        verify { accelerometerProcessorSpy wasNot Called }
    }

    @Test(expected = IllegalStateException::class)
    fun outlierThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        val randomizer = UniformRandomizer()
        val outlierThreshold = randomizer.nextDouble()
        estimator.outlierThreshold = outlierThreshold
    }

    @Test
    fun outlierThreshold_whenOutOfRange_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        // check default value
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            estimator.outlierThreshold,
            0.0
        )
        assertFalse(estimator.running)

        // set invalid values
        assertThrows(IllegalArgumentException::class.java) { estimator.outlierThreshold = -1.0 }
        assertThrows(IllegalArgumentException::class.java) { estimator.outlierThreshold = 2.0 }
    }

    @Test
    fun outlierThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        // check default value
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            estimator.outlierThreshold,
            0.0
        )
        assertFalse(estimator.running)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spy(accelerometerProcessor)
//        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        // set new value
        val randomizer = UniformRandomizer()
        val outlierThreshold = randomizer.nextDouble()

        estimator.outlierThreshold = outlierThreshold

        // check
        assertEquals(outlierThreshold, estimator.outlierThreshold, 0.0)
        assertEquals(outlierThreshold, accelerometerProcessorSpy.outlierThreshold, 0.0)
        assertEquals(outlierThreshold, gravityProcessorSpy.outlierThreshold, 0.0)

        verify(accelerometerProcessorSpy, times(1)).outlierThreshold = outlierThreshold
//        verify(exactly = 1) { accelerometerProcessorSpy.outlierThreshold = outlierThreshold }
        verify(gravityProcessorSpy, times(1)).outlierThreshold = outlierThreshold
//        verify(exactly = 1) { gravityProcessorSpy.outlierThreshold = outlierThreshold }
    }

    @Test(expected = IllegalStateException::class)
    fun outlierPanicThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        val randomizer = UniformRandomizer()
        val outlierPanicThreshold = randomizer.nextDouble()
        estimator.outlierPanicThreshold = outlierPanicThreshold
    }

    @Test
    fun outlierPanicThreshold_whenOutOfRange_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        // check default value
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.outlierPanicThreshold,
            0.0
        )
        assertFalse(estimator.running)

        // set invalid values
        assertThrows(IllegalArgumentException::class.java) {
            estimator.outlierPanicThreshold = -1.0
        }
        assertThrows(IllegalArgumentException::class.java) { estimator.outlierPanicThreshold = 2.0 }
    }

    @Test
    fun outlierPanicThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        // check default value
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.outlierPanicThreshold,
            0.0
        )
        assertFalse(estimator.running)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spy(accelerometerProcessor)
//        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        // set new value
        val randomizer = UniformRandomizer()
        val outlierPanicThreshold = randomizer.nextDouble()
        estimator.outlierPanicThreshold = outlierPanicThreshold

        // check
        assertEquals(outlierPanicThreshold, estimator.outlierPanicThreshold, 0.0)
        assertEquals(outlierPanicThreshold, accelerometerProcessorSpy.outlierPanicThreshold, 0.0)
        assertEquals(outlierPanicThreshold, gravityProcessorSpy.outlierPanicThreshold, 0.0)

        verify(accelerometerProcessorSpy, times(1)).outlierPanicThreshold = outlierPanicThreshold
/*        verify(exactly = 1) {
            accelerometerProcessorSpy.outlierPanicThreshold = outlierPanicThreshold
        }*/
        verify(gravityProcessorSpy, times(1)).outlierPanicThreshold = outlierPanicThreshold
//        verify(exactly = 1) { gravityProcessorSpy.outlierPanicThreshold = outlierPanicThreshold }
    }

    @Test(expected = IllegalStateException::class)
    fun panicCounterThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        estimator.panicCounterThreshold = 1
    }

    @Test(expected = IllegalArgumentException::class)
    fun panicCounterThreshold_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        assertFalse(estimator.running)

        estimator.panicCounterThreshold = 0
    }

    @Test
    fun panicCounterThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        assertFalse(estimator.running)
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.panicCounterThreshold
        )

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spy(accelerometerProcessor)
//        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        estimator.panicCounterThreshold = 2

        // check
        assertEquals(2, estimator.panicCounterThreshold)
        assertEquals(2, accelerometerProcessorSpy.panicCounterThreshold)
        assertEquals(2, gravityProcessorSpy.panicCounterThreshold)

        verify(accelerometerProcessorSpy, times(1)).panicCounterThreshold = 2
//        verify(exactly = 1) { accelerometerProcessorSpy.panicCounterThreshold = 2 }
        verify(gravityProcessorSpy, times(1)).panicCounterThreshold = 2
//        verify(exactly = 1) { gravityProcessorSpy.panicCounterThreshold = 2 }
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        assertFalse(estimator.running)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        estimator.start()
    }

    @Test
    fun start_whenUseAccelerometerAndInternalSyncerFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context, useAccelerometer = true)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spy(accelerometerProcessor)
//        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val accelerometerGyroscopeAndMagnetometerSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerSyncer")
        requireNotNull(accelerometerGyroscopeAndMagnetometerSyncer)

        val gravityGyroscopeAndMagnetometerSyncer: GravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityGyroscopeAndMagnetometerSyncer")
        requireNotNull(gravityGyroscopeAndMagnetometerSyncer)

        val accelerometerGyroscopeAndMagnetometerSyncerSpy =
            spy(accelerometerGyroscopeAndMagnetometerSyncer)
/*        val accelerometerGyroscopeAndMagnetometerSyncerSpy =
            spyk(accelerometerGyroscopeAndMagnetometerSyncer)*/
        doReturn(false).whenever(accelerometerGyroscopeAndMagnetometerSyncerSpy).start(any())
//        every { accelerometerGyroscopeAndMagnetometerSyncerSpy.start(any()) }.returns(false)
        val gravityGyroscopeAndMagnetometerSyncerSpy = spy(gravityGyroscopeAndMagnetometerSyncer)
//        val gravityGyroscopeAndMagnetometerSyncerSpy = spyk(gravityGyroscopeAndMagnetometerSyncer)

        estimator.setPrivateProperty(
            "accelerometerGyroscopeAndMagnetometerSyncer",
            accelerometerGyroscopeAndMagnetometerSyncerSpy
        )
        estimator.setPrivateProperty(
            "gravityGyroscopeAndMagnetometerSyncer",
            gravityGyroscopeAndMagnetometerSyncerSpy
        )

        // start
        val startTimestamp = System.nanoTime()
        assertFalse(estimator.start(startTimestamp))

        verify(accelerometerProcessorSpy, times(1)).reset()
//        verify(exactly = 1) { accelerometerProcessorSpy.reset() }
        verify(accelerometerGyroscopeAndMagnetometerSyncerSpy, only()).start(startTimestamp)
//        verify(exactly = 1) { accelerometerGyroscopeAndMagnetometerSyncerSpy.start(startTimestamp) }
        verifyNoInteractions(gravityProcessorSpy)
//        verify { gravityProcessorSpy wasNot Called }
        verifyNoInteractions(gravityGyroscopeAndMagnetometerSyncerSpy)
//        verify { gravityGyroscopeAndMagnetometerSyncerSpy wasNot Called }
    }

    @Test
    fun start_whenNotUseAccelerometerAndInternalSyncerFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context, useAccelerometer = false)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spy(accelerometerProcessor)
//        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val accelerometerGyroscopeAndMagnetometerSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerSyncer")
        requireNotNull(accelerometerGyroscopeAndMagnetometerSyncer)

        val gravityGyroscopeAndMagnetometerSyncer: GravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityGyroscopeAndMagnetometerSyncer")
        requireNotNull(gravityGyroscopeAndMagnetometerSyncer)

        val accelerometerGyroscopeAndMagnetometerSyncerSpy =
            spy(accelerometerGyroscopeAndMagnetometerSyncer)
/*        val accelerometerGyroscopeAndMagnetometerSyncerSpy =
            spyk(accelerometerGyroscopeAndMagnetometerSyncer)*/
        val gravityGyroscopeAndMagnetometerSyncerSpy = spy(gravityGyroscopeAndMagnetometerSyncer)
//        val gravityGyroscopeAndMagnetometerSyncerSpy = spyk(gravityGyroscopeAndMagnetometerSyncer)
        doReturn(false).whenever(gravityGyroscopeAndMagnetometerSyncerSpy).start(any())
//        every { gravityGyroscopeAndMagnetometerSyncerSpy.start(any()) }.returns(false)

        estimator.setPrivateProperty(
            "accelerometerGyroscopeAndMagnetometerSyncer",
            accelerometerGyroscopeAndMagnetometerSyncerSpy
        )
        estimator.setPrivateProperty(
            "gravityGyroscopeAndMagnetometerSyncer",
            gravityGyroscopeAndMagnetometerSyncerSpy
        )

        // start
        val startTimestamp = System.nanoTime()
        assertFalse(estimator.start(startTimestamp))

        verify(gravityProcessorSpy, times(1)).reset()
//        verify(exactly = 1) { gravityProcessorSpy.reset() }
        verify(gravityGyroscopeAndMagnetometerSyncerSpy, only()).start(startTimestamp)
//        verify(exactly = 1) { gravityGyroscopeAndMagnetometerSyncerSpy.start(startTimestamp) }
        verifyNoInteractions(accelerometerProcessorSpy)
//        verify { accelerometerProcessorSpy wasNot Called }
        verifyNoInteractions(accelerometerGyroscopeAndMagnetometerSyncerSpy)
//        verify { accelerometerGyroscopeAndMagnetometerSyncerSpy wasNot Called }
    }

    @Test
    fun start_whenUseAccelerometerAndInternalSyncerSucceeds_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context, useAccelerometer = true)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spy(accelerometerProcessor)
//        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val accelerometerGyroscopeAndMagnetometerSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerSyncer")
        requireNotNull(accelerometerGyroscopeAndMagnetometerSyncer)

        val gravityGyroscopeAndMagnetometerSyncer: GravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityGyroscopeAndMagnetometerSyncer")
        requireNotNull(gravityGyroscopeAndMagnetometerSyncer)

        val accelerometerGyroscopeAndMagnetometerSyncerSpy =
            spy(accelerometerGyroscopeAndMagnetometerSyncer)
/*        val accelerometerGyroscopeAndMagnetometerSyncerSpy =
            spyk(accelerometerGyroscopeAndMagnetometerSyncer)*/
        doReturn(true).whenever(accelerometerGyroscopeAndMagnetometerSyncerSpy).start(any())
//        every { accelerometerGyroscopeAndMagnetometerSyncerSpy.start(any()) }.returns(true)
        val gravityGyroscopeAndMagnetometerSyncerSpy = spy(gravityGyroscopeAndMagnetometerSyncer)
//        val gravityGyroscopeAndMagnetometerSyncerSpy = spyk(gravityGyroscopeAndMagnetometerSyncer)

        estimator.setPrivateProperty(
            "accelerometerGyroscopeAndMagnetometerSyncer",
            accelerometerGyroscopeAndMagnetometerSyncerSpy
        )
        estimator.setPrivateProperty(
            "gravityGyroscopeAndMagnetometerSyncer",
            gravityGyroscopeAndMagnetometerSyncerSpy
        )

        // start
        val startTimestamp = System.nanoTime()
        assertTrue(estimator.start(startTimestamp))

        verify(accelerometerProcessorSpy, times(1)).reset()
//        verify(exactly = 1) { accelerometerProcessorSpy.reset() }
        verify(accelerometerGyroscopeAndMagnetometerSyncerSpy, only()).start(startTimestamp)
//        verify(exactly = 1) { accelerometerGyroscopeAndMagnetometerSyncerSpy.start(startTimestamp) }
        verifyNoInteractions(gravityProcessorSpy)
//        verify { gravityProcessorSpy wasNot Called }
        verifyNoInteractions(gravityGyroscopeAndMagnetometerSyncerSpy)
//        verify { gravityGyroscopeAndMagnetometerSyncerSpy wasNot Called }
    }

    @Test
    fun start_whenNotUseAccelerometerAndInternalSyncerSucceeds_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context, useAccelerometer = false)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spy(accelerometerProcessor)
//        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val accelerometerGyroscopeAndMagnetometerSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerSyncer")
        requireNotNull(accelerometerGyroscopeAndMagnetometerSyncer)

        val gravityGyroscopeAndMagnetometerSyncer: GravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityGyroscopeAndMagnetometerSyncer")
        requireNotNull(gravityGyroscopeAndMagnetometerSyncer)

        val accelerometerGyroscopeAndMagnetometerSyncerSpy =
            spy(accelerometerGyroscopeAndMagnetometerSyncer)
/*        val accelerometerGyroscopeAndMagnetometerSyncerSpy =
            spyk(accelerometerGyroscopeAndMagnetometerSyncer)*/
        val gravityGyroscopeAndMagnetometerSyncerSpy = spy(gravityGyroscopeAndMagnetometerSyncer)
//        val gravityGyroscopeAndMagnetometerSyncerSpy = spyk(gravityGyroscopeAndMagnetometerSyncer)
        doReturn(true).whenever(gravityGyroscopeAndMagnetometerSyncerSpy).start(any())
//        every { gravityGyroscopeAndMagnetometerSyncerSpy.start(any()) }.returns(true)

        estimator.setPrivateProperty(
            "accelerometerGyroscopeAndMagnetometerSyncer",
            accelerometerGyroscopeAndMagnetometerSyncerSpy
        )
        estimator.setPrivateProperty(
            "gravityGyroscopeAndMagnetometerSyncer",
            gravityGyroscopeAndMagnetometerSyncerSpy
        )

        // start
        val startTimestamp = System.nanoTime()
        assertTrue(estimator.start(startTimestamp))

        verify(gravityProcessorSpy, times(1)).reset()
//        verify(exactly = 1) { gravityProcessorSpy.reset() }
        verify(gravityGyroscopeAndMagnetometerSyncerSpy, only()).start(startTimestamp)
//        verify(exactly = 1) { gravityGyroscopeAndMagnetometerSyncerSpy.start(startTimestamp) }
        verifyNoInteractions(accelerometerProcessorSpy)
//        verify { accelerometerProcessorSpy wasNot Called }
        verifyNoInteractions(accelerometerGyroscopeAndMagnetometerSyncerSpy)
//        verify { accelerometerGyroscopeAndMagnetometerSyncerSpy wasNot Called }
    }

    @Test
    fun stop_stopsInternalSyncersAndUpdatesRunningStatus() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        val accelerometerGyroscopeAndMagnetometerSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerSyncer")
        requireNotNull(accelerometerGyroscopeAndMagnetometerSyncer)

        val gravityGyroscopeAndMagnetometerSyncer: GravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityGyroscopeAndMagnetometerSyncer")
        requireNotNull(gravityGyroscopeAndMagnetometerSyncer)

        val accelerometerGyroscopeAndMagnetometerSyncerSpy =
            spy(accelerometerGyroscopeAndMagnetometerSyncer)
/*        val accelerometerGyroscopeAndMagnetometerSyncerSpy =
            spyk(accelerometerGyroscopeAndMagnetometerSyncer)*/
        val gravityGyroscopeAndMagnetometerSyncerSpy = spy(gravityGyroscopeAndMagnetometerSyncer)
//        val gravityGyroscopeAndMagnetometerSyncerSpy = spyk(gravityGyroscopeAndMagnetometerSyncer)

        estimator.setPrivateProperty(
            "accelerometerGyroscopeAndMagnetometerSyncer",
            accelerometerGyroscopeAndMagnetometerSyncerSpy
        )
        estimator.setPrivateProperty(
            "gravityGyroscopeAndMagnetometerSyncer",
            gravityGyroscopeAndMagnetometerSyncerSpy
        )

        // stop
        estimator.stop()

        // check
        assertFalse(estimator.running)

        verify(accelerometerGyroscopeAndMagnetometerSyncerSpy, times(1)).stop()
//        verify(exactly = 1) { accelerometerGyroscopeAndMagnetometerSyncerSpy.stop() }
        verify(gravityGyroscopeAndMagnetometerSyncerSpy, times(1)).stop()
//        verify(exactly = 1) { gravityGyroscopeAndMagnetometerSyncerSpy.stop() }
    }

    @Test
    fun gravityGyroscopeAndMagnetometerSyncer_whenAccuracyChangedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context, useAccelerometer = false)

        assertNull(estimator.accuracyChangedListener)

        val gravityGyroscopeAndMagnetometerSyncer: GravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityGyroscopeAndMagnetometerSyncer")
        requireNotNull(gravityGyroscopeAndMagnetometerSyncer)

        val listener = gravityGyroscopeAndMagnetometerSyncer.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            gravityGyroscopeAndMagnetometerSyncer,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )
    }

    @Test
    fun gravityGyroscopeAndMagnetometerSyncer_whenAccuracyChangedAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(
            context,
            useAccelerometer = false,
            accuracyChangedListener = accuracyChangedListener
        )

        assertSame(accuracyChangedListener, estimator.accuracyChangedListener)

        val gravityGyroscopeAndMagnetometerSyncer: GravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityGyroscopeAndMagnetometerSyncer")
        requireNotNull(gravityGyroscopeAndMagnetometerSyncer)

        val listener = gravityGyroscopeAndMagnetometerSyncer.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            gravityGyroscopeAndMagnetometerSyncer,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )

        verify(accuracyChangedListener, only()).onAccuracyChanged(
            estimator,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )
/*        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                estimator,
                SensorType.GRAVITY,
                SensorAccuracy.MEDIUM
            )
        }*/
    }

    @Test
    fun gravityGyroscopeAndMagnetometerSyncer_whenBufferFilledAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context, useAccelerometer = false)

        assertNull(estimator.bufferFilledListener)

        val gravityGyroscopeAndMagnetometerSyncer: GravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityGyroscopeAndMagnetometerSyncer")
        requireNotNull(gravityGyroscopeAndMagnetometerSyncer)

        val listener = gravityGyroscopeAndMagnetometerSyncer.bufferFilledListener
        requireNotNull(listener)
        listener.onBufferFilled(
            gravityGyroscopeAndMagnetometerSyncer,
            SensorType.GRAVITY
        )
    }

    @Test
    fun gravityGyroscopeAndMagnetometerSyncer_whenBufferFilledAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(
            context,
            useAccelerometer = false,
            bufferFilledListener = bufferFilledListener
        )

        assertSame(bufferFilledListener, estimator.bufferFilledListener)

        val gravityGyroscopeAndMagnetometerSyncer: GravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityGyroscopeAndMagnetometerSyncer")
        requireNotNull(gravityGyroscopeAndMagnetometerSyncer)

        val listener = gravityGyroscopeAndMagnetometerSyncer.bufferFilledListener
        requireNotNull(listener)
        listener.onBufferFilled(
            gravityGyroscopeAndMagnetometerSyncer,
            SensorType.GRAVITY
        )

        verify(bufferFilledListener, only()).onBufferFilled(
            estimator,
            SensorType.GRAVITY
        )
/*        verify(exactly = 1) {
            bufferFilledListener.onBufferFilled(
                estimator,
                SensorType.GRAVITY
            )
        }*/
    }

    @Test
    fun gravityGyroscopeAndMagnetometerSyncer_whenSyncedMeasurementAndNotProcessed_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context, useAccelerometer = false)

        assertNull(estimator.attitudeAvailableListener)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        doReturn(false).whenever(gravityProcessorSpy).process(any())
//        every { gravityProcessorSpy.process(any()) }.returns(false)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val gravityGyroscopeAndMagnetometerSyncer: GravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityGyroscopeAndMagnetometerSyncer")
        requireNotNull(gravityGyroscopeAndMagnetometerSyncer)

        val listener = gravityGyroscopeAndMagnetometerSyncer.syncedMeasurementListener
        requireNotNull(listener)
        val syncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onSyncedMeasurements(
            gravityGyroscopeAndMagnetometerSyncer,
            syncedMeasurement
        )

        // check
        verify(gravityProcessorSpy, only()).process(syncedMeasurement)
//        verify(exactly = 1) { gravityProcessorSpy.process(syncedMeasurement) }
        verify(gravityProcessorSpy, never()).fusedAttitude
//        verify(exactly = 0) { gravityProcessorSpy.fusedAttitude }
    }

    @Test
    fun gravityGyroscopeAndMagnetometerSyncer_whenSyncedMeasurementProcessedAndCoordinateTransformationAndEulerAnglesEstimatedAndNoListener_makesEstimation() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(
            context,
            useAccelerometer = false,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = true
        )

        assertNull(estimator.attitudeAvailableListener)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        doReturn(true).whenever(gravityProcessorSpy).process(any())
//        every { gravityProcessorSpy.process(any()) }.returns(true)
        val attitude = getAttitude()
        doReturn(attitude).whenever(gravityProcessorSpy).fusedAttitude
//        every { gravityProcessorSpy.fusedAttitude }.returns(attitude)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val gravityGyroscopeAndMagnetometerSyncer: GravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityGyroscopeAndMagnetometerSyncer")
        requireNotNull(gravityGyroscopeAndMagnetometerSyncer)

        val listener = gravityGyroscopeAndMagnetometerSyncer.syncedMeasurementListener
        requireNotNull(listener)
        val syncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onSyncedMeasurements(
            gravityGyroscopeAndMagnetometerSyncer,
            syncedMeasurement
        )

        // check
        verify(gravityProcessorSpy, times(1)).process(syncedMeasurement)
//        verify(exactly = 1) { gravityProcessorSpy.process(syncedMeasurement) }
        verify(gravityProcessorSpy, times(1)).fusedAttitude
//        verify(exactly = 1) { gravityProcessorSpy.fusedAttitude }

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)
        assertEquals(attitude, fusedAttitude)
        assertNotSame(attitude, fusedAttitude)

        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        requireNotNull(coordinateTransformation)
        assertEquals(
            CoordinateTransformation(
                attitude,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            ), coordinateTransformation
        )

        val eulerAngles: DoubleArray? =
            estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles)
        assertArrayEquals(attitude.toEulerAngles(), eulerAngles, 0.0)
    }

    @Test
    fun gravityGyroscopeAndMagnetometerSyncer_whenSyncedMeasurementProcessedAndCoordinateTransformationAndEulerAnglesEstimatedAndListener_makesEstimationAndNotifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(
            context,
            useAccelerometer = false,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = true,
            attitudeAvailableListener = attitudeListener
        )

        assertSame(attitudeListener, estimator.attitudeAvailableListener)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spy(gravityProcessor)
//        val gravityProcessorSpy = spyk(gravityProcessor)
        doReturn(true).whenever(gravityProcessorSpy).process(any())
//        every { gravityProcessorSpy.process(any()) }.returns(true)
        val attitude = getAttitude()
        doReturn(attitude).whenever(gravityProcessorSpy).fusedAttitude
//        every { gravityProcessorSpy.fusedAttitude }.returns(attitude)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val gravityGyroscopeAndMagnetometerSyncer: GravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityGyroscopeAndMagnetometerSyncer")
        requireNotNull(gravityGyroscopeAndMagnetometerSyncer)

        val listener = gravityGyroscopeAndMagnetometerSyncer.syncedMeasurementListener
        requireNotNull(listener)
        val timestamp = System.nanoTime()
        val syncedMeasurement =
            GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(timestamp = timestamp)
        listener.onSyncedMeasurements(
            gravityGyroscopeAndMagnetometerSyncer,
            syncedMeasurement
        )

        // check
        verify(gravityProcessorSpy, times(1)).process(syncedMeasurement)
//        verify(exactly = 1) { gravityProcessorSpy.process(syncedMeasurement) }
        verify(gravityProcessorSpy, times(1)).fusedAttitude
//        verify(exactly = 1) { gravityProcessorSpy.fusedAttitude }

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)
        assertEquals(attitude, fusedAttitude)
        assertNotSame(attitude, fusedAttitude)

        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        requireNotNull(coordinateTransformation)
        assertEquals(
            CoordinateTransformation(
                attitude,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            ), coordinateTransformation
        )

        val eulerAngles: DoubleArray? =
            estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles)
        assertArrayEquals(attitude.toEulerAngles(), eulerAngles, 0.0)

        verify(attitudeListener, only()).onAttitudeAvailable(
            estimator,
            fusedAttitude,
            timestamp,
            eulerAngles[0],
            eulerAngles[1],
            eulerAngles[2],
            coordinateTransformation
        )
/*        verify(exactly = 1) {
            attitudeListener.onAttitudeAvailable(
                estimator,
                fusedAttitude,
                timestamp,
                eulerAngles[0],
                eulerAngles[1],
                eulerAngles[2],
                coordinateTransformation
            )
        }*/
    }

    @Test
    fun gravityGyroscopeAndMagnetometerSyncer_whenSyncedMeasurementProcessedAndCoordinateTransformationAndEulerAnglesNotEstimatedAndListener_makesEstimationAndNotifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeListener
        )

        assertSame(attitudeListener, estimator.attitudeAvailableListener)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spy(accelerometerProcessor)
//        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        doReturn(true).whenever(accelerometerProcessorSpy).process(any())
//        every { accelerometerProcessorSpy.process(any()) }.returns(true)
        val attitude = getAttitude()
        doReturn(attitude).whenever(accelerometerProcessorSpy).fusedAttitude
//        every { accelerometerProcessorSpy.fusedAttitude }.returns(attitude)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val accelerometerGyroscopeAndMagnetometerSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerSyncer")
        requireNotNull(accelerometerGyroscopeAndMagnetometerSyncer)

        val listener = accelerometerGyroscopeAndMagnetometerSyncer.syncedMeasurementListener
        requireNotNull(listener)
        val timestamp = System.nanoTime()
        val syncedMeasurement =
            AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement(timestamp = timestamp)
        listener.onSyncedMeasurements(
            accelerometerGyroscopeAndMagnetometerSyncer,
            syncedMeasurement
        )

        // check
        verify(accelerometerProcessorSpy, times(1)).process(syncedMeasurement)
//        verify(exactly = 1) { accelerometerProcessorSpy.process(syncedMeasurement) }
        verify(accelerometerProcessorSpy, times(1)).fusedAttitude
//        verify(exactly = 1) { accelerometerProcessorSpy.fusedAttitude }

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)
        assertEquals(attitude, fusedAttitude)
        assertNotSame(attitude, fusedAttitude)

        verify(attitudeListener, only()).onAttitudeAvailable(
            estimator,
            fusedAttitude,
            timestamp,
            null,
            null,
            null,
            null
        )
/*        verify(exactly = 1) {
            attitudeListener.onAttitudeAvailable(
                estimator,
                fusedAttitude,
                timestamp,
                null,
                null,
                null,
                null
            )
        }*/
    }

    @Test
    fun accelerometerGyroscopeAndMagnetometerSyncer_whenAccuracyChangedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context, useAccelerometer = true)

        assertNull(estimator.accuracyChangedListener)

        val accelerometerGyroscopeAndMagnetometerSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerSyncer")
        requireNotNull(accelerometerGyroscopeAndMagnetometerSyncer)

        val listener = accelerometerGyroscopeAndMagnetometerSyncer.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            accelerometerGyroscopeAndMagnetometerSyncer,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )
    }

    @Test
    fun accelerometerGyroscopeAndMagnetometerSyncer_whenAccuracyChangedAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(
            context,
            useAccelerometer = true,
            accuracyChangedListener = accuracyChangedListener
        )

        assertSame(accuracyChangedListener, estimator.accuracyChangedListener)

        val accelerometerGyroscopeAndMagnetometerSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerSyncer")
        requireNotNull(accelerometerGyroscopeAndMagnetometerSyncer)

        val listener = accelerometerGyroscopeAndMagnetometerSyncer.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            accelerometerGyroscopeAndMagnetometerSyncer,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )

        verify(accuracyChangedListener, only()).onAccuracyChanged(
            estimator,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )
/*        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                estimator,
                SensorType.GRAVITY,
                SensorAccuracy.MEDIUM
            )
        }*/
    }

    @Test
    fun accelerometerGyroscopeAndMagnetometerSyncer_whenBufferFilledAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context, useAccelerometer = true)

        assertNull(estimator.bufferFilledListener)

        val accelerometerGyroscopeAndMagnetometerSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerSyncer")
        requireNotNull(accelerometerGyroscopeAndMagnetometerSyncer)

        val listener = accelerometerGyroscopeAndMagnetometerSyncer.bufferFilledListener
        requireNotNull(listener)
        listener.onBufferFilled(
            accelerometerGyroscopeAndMagnetometerSyncer,
            SensorType.GRAVITY
        )
    }

    @Test
    fun accelerometerGyroscopeAndMagnetometerSyncer_whenBufferFilledAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(
            context,
            useAccelerometer = true,
            bufferFilledListener = bufferFilledListener
        )

        assertSame(bufferFilledListener, estimator.bufferFilledListener)

        val accelerometerGyroscopeAndMagnetometerSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerSyncer")
        requireNotNull(accelerometerGyroscopeAndMagnetometerSyncer)

        val listener = accelerometerGyroscopeAndMagnetometerSyncer.bufferFilledListener
        requireNotNull(listener)
        listener.onBufferFilled(
            accelerometerGyroscopeAndMagnetometerSyncer,
            SensorType.GRAVITY
        )

        verify(bufferFilledListener, only()).onBufferFilled(
            estimator,
            SensorType.GRAVITY
        )
/*        verify(exactly = 1) {
            bufferFilledListener.onBufferFilled(
                estimator,
                SensorType.GRAVITY
            )
        }*/
    }

    @Test
    fun accelerometerGyroscopeAndMagnetometerSyncer_whenSyncedMeasurementAndNotProcessed_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(context, useAccelerometer = true)

        assertNull(estimator.attitudeAvailableListener)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spy(accelerometerProcessor)
//        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        doReturn(false).whenever(accelerometerProcessorSpy).process(any())
//        every { accelerometerProcessorSpy.process(any()) }.returns(false)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val accelerometerGyroscopeAndMagnetometerSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerSyncer")
        requireNotNull(accelerometerGyroscopeAndMagnetometerSyncer)

        val listener = accelerometerGyroscopeAndMagnetometerSyncer.syncedMeasurementListener
        requireNotNull(listener)
        val syncedMeasurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onSyncedMeasurements(
            accelerometerGyroscopeAndMagnetometerSyncer,
            syncedMeasurement
        )

        // check
        verify(accelerometerProcessorSpy, only()).process(syncedMeasurement)
//        verify(exactly = 1) { accelerometerProcessorSpy.process(syncedMeasurement) }
        verify(accelerometerProcessorSpy, never()).fusedAttitude
//        verify(exactly = 0) { accelerometerProcessorSpy.fusedAttitude }
    }

    @Test
    fun accelerometerGyroscopeAndMagnetometerSyncer_whenSyncedMeasurementProcessedAndCoordinateTransformationAndEulerAnglesEstimatedAndNoListener_makesEstimation() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(
            context,
            useAccelerometer = true,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = true
        )

        assertNull(estimator.attitudeAvailableListener)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spy(accelerometerProcessor)
//        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        doReturn(true).whenever(accelerometerProcessorSpy).process(any())
//        every { accelerometerProcessorSpy.process(any()) }.returns(true)
        val attitude = getAttitude()
        doReturn(attitude).whenever(accelerometerProcessorSpy).fusedAttitude
//        every { accelerometerProcessorSpy.fusedAttitude }.returns(attitude)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val accelerometerGyroscopeAndMagnetometerSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerSyncer")
        requireNotNull(accelerometerGyroscopeAndMagnetometerSyncer)

        val listener = accelerometerGyroscopeAndMagnetometerSyncer.syncedMeasurementListener
        requireNotNull(listener)
        val syncedMeasurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onSyncedMeasurements(
            accelerometerGyroscopeAndMagnetometerSyncer,
            syncedMeasurement
        )

        // check
        verify(accelerometerProcessorSpy, times(1)).process(syncedMeasurement)
//        verify(exactly = 1) { accelerometerProcessorSpy.process(syncedMeasurement) }
        verify(accelerometerProcessorSpy, times(1)).fusedAttitude
//        verify(exactly = 1) { accelerometerProcessorSpy.fusedAttitude }

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)
        assertEquals(attitude, fusedAttitude)
        assertNotSame(attitude, fusedAttitude)

        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        requireNotNull(coordinateTransformation)
        assertEquals(
            CoordinateTransformation(
                attitude,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            ), coordinateTransformation
        )

        val eulerAngles: DoubleArray? =
            estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles)
        assertArrayEquals(attitude.toEulerAngles(), eulerAngles, 0.0)
    }

    @Test
    fun accelerometerGyroscopeAndMagnetometerSyncer_whenSyncedMeasurementProcessedAndCoordinateTransformationAndEulerAnglesEstimatedAndListener_makesEstimationAndNotifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(
            context,
            useAccelerometer = true,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = true,
            attitudeAvailableListener = attitudeListener
        )

        assertSame(attitudeListener, estimator.attitudeAvailableListener)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spy(accelerometerProcessor)
//        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        doReturn(true).whenever(accelerometerProcessorSpy).process(any())
//        every { accelerometerProcessorSpy.process(any()) }.returns(true)
        val attitude = getAttitude()
        doReturn(attitude).whenever(accelerometerProcessorSpy).fusedAttitude
//        every { accelerometerProcessorSpy.fusedAttitude }.returns(attitude)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val accelerometerGyroscopeAndMagnetometerSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerSyncer")
        requireNotNull(accelerometerGyroscopeAndMagnetometerSyncer)

        val listener = accelerometerGyroscopeAndMagnetometerSyncer.syncedMeasurementListener
        requireNotNull(listener)
        val timestamp = System.nanoTime()
        val syncedMeasurement =
            AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement(timestamp = timestamp)
        listener.onSyncedMeasurements(
            accelerometerGyroscopeAndMagnetometerSyncer,
            syncedMeasurement
        )

        // check
        verify(accelerometerProcessorSpy, times(1)).process(syncedMeasurement)
//        verify(exactly = 1) { accelerometerProcessorSpy.process(syncedMeasurement) }
        verify(accelerometerProcessorSpy, times(1)).fusedAttitude
//        verify(exactly = 1) { accelerometerProcessorSpy.fusedAttitude }

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)
        assertEquals(attitude, fusedAttitude)
        assertNotSame(attitude, fusedAttitude)

        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        requireNotNull(coordinateTransformation)
        assertEquals(
            CoordinateTransformation(
                attitude,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            ), coordinateTransformation
        )

        val eulerAngles: DoubleArray? =
            estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles)
        assertArrayEquals(attitude.toEulerAngles(), eulerAngles, 0.0)

        verify(attitudeListener, only()).onAttitudeAvailable(
            estimator,
            fusedAttitude,
            timestamp,
            eulerAngles[0],
            eulerAngles[1],
            eulerAngles[2],
            coordinateTransformation
        )
/*        verify(exactly = 1) {
            attitudeListener.onAttitudeAvailable(
                estimator,
                fusedAttitude,
                timestamp,
                eulerAngles[0],
                eulerAngles[1],
                eulerAngles[2],
                coordinateTransformation
            )
        }*/
    }

    @Test
    fun accelerometerGyroscopeAndMagnetometerSyncer_whenSyncedMeasurementProcessedAndCoordinateTransformationAndEulerAnglesNotEstimatedAndListener_makesEstimationAndNotifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator2(
            context,
            useAccelerometer = true,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeListener
        )

        assertSame(attitudeListener, estimator.attitudeAvailableListener)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spy(accelerometerProcessor)
//        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        doReturn(true).whenever(accelerometerProcessorSpy).process(any())
//        every { accelerometerProcessorSpy.process(any()) }.returns(true)
        val attitude = getAttitude()
        doReturn(attitude).whenever(accelerometerProcessorSpy).fusedAttitude
//        every { accelerometerProcessorSpy.fusedAttitude }.returns(attitude)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val accelerometerGyroscopeAndMagnetometerSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerSyncer")
        requireNotNull(accelerometerGyroscopeAndMagnetometerSyncer)

        val listener = accelerometerGyroscopeAndMagnetometerSyncer.syncedMeasurementListener
        requireNotNull(listener)
        val timestamp = System.nanoTime()
        val syncedMeasurement =
            AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement(timestamp = timestamp)
        listener.onSyncedMeasurements(
            accelerometerGyroscopeAndMagnetometerSyncer,
            syncedMeasurement
        )

        // check
        verify(accelerometerProcessorSpy, times(1)).process(syncedMeasurement)
//        verify(exactly = 1) { accelerometerProcessorSpy.process(syncedMeasurement) }
        verify(accelerometerProcessorSpy, times(1)).fusedAttitude
//        verify(exactly = 1) { accelerometerProcessorSpy.fusedAttitude }

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)
        assertEquals(attitude, fusedAttitude)
        assertNotSame(attitude, fusedAttitude)

        verify(attitudeListener, only()).onAttitudeAvailable(
            estimator,
            fusedAttitude,
            timestamp,
            null,
            null,
            null,
            null
        )
/*        verify(exactly = 1) {
            attitudeListener.onAttitudeAvailable(
                estimator,
                fusedAttitude,
                timestamp,
                null,
                null,
                null,
                null
            )
        }*/
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