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
package com.irurueta.android.navigation.inertial.estimators.attitude

import android.content.Context
import android.hardware.SensorManager
import android.location.Location
import android.os.SystemClock
import androidx.test.core.app.ApplicationProvider
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.estimators.filter.MeanAveragingFilter
import com.irurueta.android.navigation.inertial.processors.attitude.AccelerometerGravityProcessor
import com.irurueta.android.navigation.inertial.processors.attitude.BaseLevelingProcessor
import com.irurueta.android.navigation.inertial.processors.attitude.GravityProcessor
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator
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
import kotlin.math.sqrt

@Ignore("possible memory leak")
@RunWith(RobolectricTestRunner::class)
class LevelingEstimator2Test {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var levelingAvailableListener: LevelingEstimator2.OnLevelingAvailableListener

    @MockK(relaxUnitFun = true)
    private lateinit var accuracyChangedListener: LevelingEstimator2.OnAccuracyChangedListener

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
        val estimator = LevelingEstimator2(context)

        // check
        assertSame(context, estimator.context)
        assertEquals(SensorDelay.GAME, estimator.sensorDelay)
        assertTrue(estimator.useAccelerometer)
        assertTrue(estimator.startOffsetEnabled)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.accelerometerSensorType
        )
        assertNotNull(estimator.accelerometerAveragingFilter)
        assertFalse(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateEulerAngles)
        assertNull(estimator.levelingAvailableListener)
        assertNull(estimator.accuracyChangedListener)
        assertNull(estimator.location)
        assertTrue(estimator.adjustGravityNorm)
        assertFalse(estimator.running)
        assertNull(estimator.startOffset)

        val gravityProcessor: GravityProcessor? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "gravityProcessor")
        requireNotNull(gravityProcessor)
        assertNull(gravityProcessor.location)
        assertTrue(gravityProcessor.adjustGravityNorm)

        val accelerometerGravityProcessor: AccelerometerGravityProcessor? =
            getPrivateProperty(
                BaseLevelingEstimator2::class,
                estimator,
                "accelerometerGravityProcessor"
            )
        requireNotNull(accelerometerGravityProcessor)
        assertNull(accelerometerGravityProcessor.location)
        assertTrue(accelerometerGravityProcessor.adjustGravityNorm)
    }

    @Test
    fun constructor_whenAllProperties_setsExpectedValues() {
        val location = getLocation()
        val accelerometerAveragingFilter = MeanAveragingFilter()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator2(
            context,
            SensorDelay.NORMAL,
            useAccelerometer = false,
            startOffsetEnabled = false,
            AccelerometerSensorType.ACCELEROMETER,
            accelerometerAveragingFilter,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = false,
            levelingAvailableListener,
            accuracyChangedListener,
            location,
            adjustGravityNorm = false
        )

        // check
        assertSame(context, estimator.context)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertFalse(estimator.useAccelerometer)
        assertFalse(estimator.startOffsetEnabled)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            estimator.accelerometerSensorType
        )
        assertSame(accelerometerAveragingFilter, estimator.accelerometerAveragingFilter)
        assertTrue(estimator.estimateCoordinateTransformation)
        assertFalse(estimator.estimateEulerAngles)
        assertSame(levelingAvailableListener, estimator.levelingAvailableListener)
        assertSame(accuracyChangedListener, estimator.accuracyChangedListener)
        assertSame(location, estimator.location)
        assertFalse(estimator.adjustGravityNorm)
        assertFalse(estimator.running)
        assertNull(estimator.startOffset)

        val gravityProcessor: GravityProcessor? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "gravityProcessor")
        requireNotNull(gravityProcessor)
        assertSame(location, gravityProcessor.location)
        assertFalse(gravityProcessor.adjustGravityNorm)

        val accelerometerGravityProcessor: AccelerometerGravityProcessor? =
            getPrivateProperty(
                BaseLevelingEstimator2::class,
                estimator,
                "accelerometerGravityProcessor"
            )
        requireNotNull(accelerometerGravityProcessor)
        assertSame(location, accelerometerGravityProcessor.location)
        assertFalse(accelerometerGravityProcessor.adjustGravityNorm)
    }

    @Test
    fun levelingAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator2(context)

        // check default value
        assertNull(estimator.levelingAvailableListener)

        // set new value
        estimator.levelingAvailableListener = levelingAvailableListener

        // check
        assertSame(levelingAvailableListener, estimator.levelingAvailableListener)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator2(context)

        // check default value
        assertNull(estimator.accuracyChangedListener)

        // set new value
        estimator.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, estimator.accuracyChangedListener)
    }

    @Test
    fun location_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location1 = getLocation()
        val estimator = LevelingEstimator2(context, location = location1)

        // check initial value
        assertSame(location1, estimator.location)

        // set new value
        val location2 = getLocation()
        estimator.location = location2

        // check
        assertSame(location2, estimator.location)

        val gravityProcessor: GravityProcessor? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "gravityProcessor")
        requireNotNull(gravityProcessor)
        assertSame(location2, gravityProcessor.location)

        val accelerometerGravityProcessor: AccelerometerGravityProcessor? =
            getPrivateProperty(
                BaseLevelingEstimator2::class,
                estimator,
                "accelerometerGravityProcessor"
            )
        requireNotNull(accelerometerGravityProcessor)
        assertSame(location2, accelerometerGravityProcessor.location)
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
    fun running_whenAccelerometerUsed_getsAccelerometerCollectorValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator2(context, useAccelerometer = true)

        val accelerometerSensorCollector: AccelerometerSensorCollector2? =
            getPrivateProperty(
                BaseLevelingEstimator2::class,
                estimator,
                "accelerometerSensorCollector"
            )
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.running }.returns(true)
        setPrivateProperty(
            BaseLevelingEstimator2::class,
            estimator,
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        val gravitySensorCollector: GravitySensorCollector2? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        setPrivateProperty(
            BaseLevelingEstimator2::class,
            estimator,
            "gravitySensorCollector",
            gravitySensorCollectorSpy
        )

        assertTrue(estimator.running)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.running }
        verify { gravitySensorCollectorSpy wasNot Called }
    }

    @Test
    fun running_whenAccelerometerNotUsed_getsGravityCollectorValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator2(context, useAccelerometer = false)

        val accelerometerSensorCollector: AccelerometerSensorCollector2? =
            getPrivateProperty(
                BaseLevelingEstimator2::class,
                estimator,
                "accelerometerSensorCollector"
            )
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        setPrivateProperty(
            BaseLevelingEstimator2::class,
            estimator,
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        val gravitySensorCollector: GravitySensorCollector2? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        every { gravitySensorCollectorSpy.running }.returns(true)
        setPrivateProperty(
            BaseLevelingEstimator2::class,
            estimator,
            "gravitySensorCollector",
            gravitySensorCollectorSpy
        )

        assertTrue(estimator.running)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.running }
        verify(exactly = 1) { gravitySensorCollectorSpy.running }
    }

    @Test
    fun start_whenAccelerometerUsed_startsAccelerometerSensorCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator2(context, useAccelerometer = true)

        val accelerometerSensorCollector: AccelerometerSensorCollector2? =
            getPrivateProperty(
                BaseLevelingEstimator2::class,
                estimator,
                "accelerometerSensorCollector"
            )
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start() }.returns(true)
        setPrivateProperty(
            BaseLevelingEstimator2::class,
            estimator,
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        val gravitySensorCollector: GravitySensorCollector2? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        setPrivateProperty(
            BaseLevelingEstimator2::class,
            estimator,
            "gravitySensorCollector",
            gravitySensorCollectorSpy
        )

        assertTrue(estimator.start())

        verify(exactly = 1) { accelerometerSensorCollectorSpy.running }
        verify(exactly = 1) { gravitySensorCollectorSpy.running }
        verify(exactly = 1) { accelerometerSensorCollectorSpy.start() }
        verify(exactly = 0) { gravitySensorCollectorSpy.start() }
    }

    @Test
    fun start_whenAccelerometerNotUsed_startsGravitySensorCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator2(context, useAccelerometer = false)

        val accelerometerSensorCollector: AccelerometerSensorCollector2? =
            getPrivateProperty(
                BaseLevelingEstimator2::class,
                estimator,
                "accelerometerSensorCollector"
            )
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        setPrivateProperty(
            BaseLevelingEstimator2::class,
            estimator,
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        val gravitySensorCollector: GravitySensorCollector2? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        every { gravitySensorCollectorSpy.start() }.returns(true)
        setPrivateProperty(
            BaseLevelingEstimator2::class,
            estimator,
            "gravitySensorCollector",
            gravitySensorCollectorSpy
        )

        assertTrue(estimator.start())

        verify(exactly = 1) { accelerometerSensorCollectorSpy.running }
        verify(exactly = 1) { gravitySensorCollectorSpy.running }
        verify(exactly = 1) { gravitySensorCollectorSpy.start() }
        verify(exactly = 0) { accelerometerSensorCollectorSpy.start() }
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator2(context, useAccelerometer = true)

        val accelerometerSensorCollector: AccelerometerSensorCollector2? =
            getPrivateProperty(
                BaseLevelingEstimator2::class,
                estimator,
                "accelerometerSensorCollector"
            )
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.running }.returns(true)
        setPrivateProperty(
            BaseLevelingEstimator2::class,
            estimator,
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        assertTrue(estimator.running)
        estimator.start()
    }

    @Test
    fun stop_stopsCollectors() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator2(context)

        val accelerometerSensorCollector: AccelerometerSensorCollector2? =
            getPrivateProperty(
                BaseLevelingEstimator2::class,
                estimator,
                "accelerometerSensorCollector"
            )
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        setPrivateProperty(
            BaseLevelingEstimator2::class,
            estimator,
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        val gravitySensorCollector: GravitySensorCollector2? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        setPrivateProperty(
            BaseLevelingEstimator2::class,
            estimator,
            "gravitySensorCollector",
            gravitySensorCollectorSpy
        )

        estimator.stop()

        verify(exactly = 1) { accelerometerSensorCollectorSpy.stop() }
        verify(exactly = 1) { gravitySensorCollectorSpy.stop() }
    }

    @Test
    fun onGravityAccuracyChanged_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator2(context)

        val gravitySensorCollector: GravitySensorCollector2? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "gravitySensorCollector")
        requireNotNull(gravitySensorCollector)

        val listener = gravitySensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(gravitySensorCollector, SensorAccuracy.HIGH)
    }

    @Test
    fun onGravityAccuracyChanged_whenListenerAvailable_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator2(
            context,
            useAccelerometer = false,
            accuracyChangedListener = accuracyChangedListener
        )

        val gravitySensorCollector: GravitySensorCollector2? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "gravitySensorCollector")
        requireNotNull(gravitySensorCollector)

        val listener = gravitySensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(gravitySensorCollector, SensorAccuracy.HIGH)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                estimator,
                SensorType.GRAVITY,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun onGravityEstimation_whenNoListenerNotEstimateCoordinateTransformationAndNotEstimateEulerAngles_updatesAttitude() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator2(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spyk(attitude)
        setPrivateProperty(BaseLevelingEstimator2::class, estimator, "attitude", attitudeSpy)

        val eulerAngles: DoubleArray? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "eulerAngles")
        requireNotNull(eulerAngles)

        val coordinateTransformation: CoordinateTransformation? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "coordinateTransformation")
        requireNotNull(coordinateTransformation)
        assertEquals(FrameType.BODY_FRAME, coordinateTransformation.sourceType)
        assertEquals(
            FrameType.LOCAL_NAVIGATION_FRAME,
            coordinateTransformation.destinationType
        )
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        setPrivateProperty(
            BaseLevelingEstimator2::class,
            estimator,
            "coordinateTransformation",
            coordinateTransformationSpy
        )

        val randomizer = UniformRandomizer()
        val latitude =
            Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES))
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        // body attitude
        val roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val yaw1 = 0.0

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        val bodyC = CoordinateTransformation(
            roll1,
            pitch1,
            yaw1,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )

        // obtain specific force neglecting north component of gravity
        val cnb = bodyC.matrix
        val nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(latitude, height)
        val g = Matrix.newFromArray(nedGravity.asArray())
        g.multiplyByScalar(-1.0)
        val f = cnb.multiplyAndReturnNew(g)

        val fx = f.getElementAtIndex(0)
        val fy = f.getElementAtIndex(1)
        val fz = f.getElementAtIndex(2)

        val floatFx = fx.toFloat()
        val floatFy = fy.toFloat()
        val floatFz = fz.toFloat()

        val gravitySensorCollector: GravitySensorCollector2? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val listener = gravitySensorCollector.measurementListener
        requireNotNull(listener)

        val gravityProcessor: GravityProcessor? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        setPrivateProperty(
            BaseLevelingEstimator2::class,
            estimator,
            "gravityProcessor",
            gravityProcessorSpy
        )

        val levelingProcessor: BaseLevelingProcessor? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "levelingProcessor")
        requireNotNull(levelingProcessor)
        val levelingProcessorSpy = spyk(levelingProcessor)
        estimator.setPrivateProperty("levelingProcessor", levelingProcessorSpy)

        val timestamp = SystemClock.elapsedRealtimeNanos()
        val measurement = GravitySensorMeasurement(
            floatFy,
            floatFx,
            -floatFz,
            timestamp,
            SensorAccuracy.MEDIUM
        )
        val norm = sqrt(
            floatFx.toDouble() * floatFx.toDouble()
                    + floatFy.toDouble() * floatFy.toDouble() + floatFz.toDouble() * floatFz.toDouble()
        )
        val factor = SensorManager.GRAVITY_EARTH / norm
        listener.onMeasurement(gravitySensorCollector, measurement)

        verify(exactly = 1) { gravityProcessorSpy.process(measurement) }
        verify(exactly = 1) {
            levelingProcessorSpy.process(
                floatFx.toDouble() * factor,
                floatFy.toDouble() * factor,
                floatFz.toDouble() * factor
            )
        }
        verify(exactly = 1) { attitudeSpy.fromQuaternion(levelingProcessor.attitude) }

        val attitude2 = Quaternion()
        bodyC.asRotation(attitude2)
        attitude2.normalize()
        attitude2.inverse()
        attitude2.normalize()
        assertTrue(attitudeSpy.equals(attitude2, ABSOLUTE_ERROR))

        verify { coordinateTransformationSpy wasNot Called }

        assertArrayEquals(eulerAngles, doubleArrayOf(0.0, 0.0, 0.0), 0.0)
    }

    @Test
    fun onGravityEstimation_whenListenerNotEstimateCoordinateTransformationAndNotEstimateEulerAngles_updatesAttitude() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator2(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            levelingAvailableListener = levelingAvailableListener
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spyk(attitude)
        setPrivateProperty(BaseLevelingEstimator2::class, estimator, "attitude", attitudeSpy)

        val eulerAngles: DoubleArray? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "eulerAngles")
        requireNotNull(eulerAngles)

        val coordinateTransformation: CoordinateTransformation? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "coordinateTransformation")
        requireNotNull(coordinateTransformation)
        assertEquals(FrameType.BODY_FRAME, coordinateTransformation.sourceType)
        assertEquals(
            FrameType.LOCAL_NAVIGATION_FRAME,
            coordinateTransformation.destinationType
        )
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        setPrivateProperty(
            BaseLevelingEstimator2::class,
            estimator,
            "coordinateTransformation",
            coordinateTransformationSpy
        )

        val randomizer = UniformRandomizer()
        val latitude =
            Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES))
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        // body attitude
        val roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        val bodyC = CoordinateTransformation(
            roll1,
            pitch1,
            yaw1,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )

        // obtain specific force neglecting north component of gravity
        val cnb = bodyC.matrix
        val nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(latitude, height)
        val g = Matrix.newFromArray(nedGravity.asArray())
        g.multiplyByScalar(-1.0)
        val f = cnb.multiplyAndReturnNew(g)

        val fx = f.getElementAtIndex(0)
        val fy = f.getElementAtIndex(1)
        val fz = f.getElementAtIndex(2)

        val floatFx = fx.toFloat()
        val floatFy = fy.toFloat()
        val floatFz = fz.toFloat()

        val gravitySensorCollector: GravitySensorCollector2? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val listener = gravitySensorCollector.measurementListener
        requireNotNull(listener)

        val gravityProcessor: GravityProcessor? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        setPrivateProperty(
            BaseLevelingEstimator2::class,
            estimator,
            "gravityProcessor",
            gravityProcessorSpy
        )

        val levelingProcessor: BaseLevelingProcessor? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "levelingProcessor")
        requireNotNull(levelingProcessor)
        val levelingProcessorSpy = spyk(levelingProcessor)
        estimator.setPrivateProperty("levelingProcessor", levelingProcessorSpy)

        val timestamp = SystemClock.elapsedRealtimeNanos()
        val measurement = GravitySensorMeasurement(
            floatFy,
            floatFx,
            -floatFz,
            timestamp,
            SensorAccuracy.MEDIUM
        )
        val norm = sqrt(
            floatFx.toDouble() * floatFx.toDouble()
                    + floatFy.toDouble() * floatFy.toDouble() + floatFz.toDouble() * floatFz.toDouble()
        )
        val factor = SensorManager.GRAVITY_EARTH / norm
        listener.onMeasurement(gravitySensorCollector, measurement)

        verify(exactly = 1) { gravityProcessorSpy.process(measurement) }
        verify(exactly = 1) {
            levelingProcessorSpy.process(
                floatFx.toDouble() * factor,
                floatFy.toDouble() * factor,
                floatFz.toDouble() * factor
            )
        }
        verify(exactly = 1) { attitudeSpy.fromQuaternion(levelingProcessor.attitude) }

        verify { coordinateTransformationSpy wasNot Called }

        assertArrayEquals(eulerAngles, doubleArrayOf(0.0, 0.0, 0.0), 0.0)

        verify(exactly = 1) {
            levelingAvailableListener.onLevelingAvailable(
                estimator,
                attitudeSpy,
                timestamp,
                null,
                null,
                null
            )
        }
    }

    @Test
    fun onGravityEstimation_whenListenerEstimateCoordinateTransformationAndEstimateEulerAngles_updatesAttitude() {
        val context = ApplicationProvider.getApplicationContext<Context>()

        val estimator = LevelingEstimator2(
            context,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = true,
            levelingAvailableListener = levelingAvailableListener
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spyk(attitude)
        setPrivateProperty(BaseLevelingEstimator2::class, estimator, "attitude", attitudeSpy)

        val eulerAngles: DoubleArray? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "eulerAngles")
        requireNotNull(eulerAngles)

        val coordinateTransformation: CoordinateTransformation? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "coordinateTransformation")
        requireNotNull(coordinateTransformation)
        assertEquals(FrameType.BODY_FRAME, coordinateTransformation.sourceType)
        assertEquals(
            FrameType.LOCAL_NAVIGATION_FRAME,
            coordinateTransformation.destinationType
        )
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        setPrivateProperty(
            BaseLevelingEstimator2::class,
            estimator,
            "coordinateTransformation",
            coordinateTransformationSpy
        )

        val randomizer = UniformRandomizer()
        val latitude =
            Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES))
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        // body attitude
        val roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        val bodyC = CoordinateTransformation(
            roll1,
            pitch1,
            yaw1,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )

        // obtain specific force neglecting north component of gravity
        val cnb = bodyC.matrix
        val nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(latitude, height)
        val g = Matrix.newFromArray(nedGravity.asArray())
        g.multiplyByScalar(-1.0)
        val f = cnb.multiplyAndReturnNew(g)

        val fx = f.getElementAtIndex(0)
        val fy = f.getElementAtIndex(1)
        val fz = f.getElementAtIndex(2)

        val floatFx = fx.toFloat()
        val floatFy = fy.toFloat()
        val floatFz = fz.toFloat()

        val gravitySensorCollector: GravitySensorCollector2? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val listener = gravitySensorCollector.measurementListener
        requireNotNull(listener)

        val gravityProcessor: GravityProcessor? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        setPrivateProperty(
            BaseLevelingEstimator2::class,
            estimator,
            "gravityProcessor",
            gravityProcessorSpy
        )

        val levelingProcessor: BaseLevelingProcessor? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "levelingProcessor")
        requireNotNull(levelingProcessor)
        val levelingProcessorSpy = spyk(levelingProcessor)
        estimator.setPrivateProperty("levelingProcessor", levelingProcessorSpy)

        val timestamp = SystemClock.elapsedRealtimeNanos()
        val measurement = GravitySensorMeasurement(
            floatFy,
            floatFx,
            -floatFz,
            timestamp,
            SensorAccuracy.MEDIUM
        )
        val norm = sqrt(
            floatFx.toDouble() * floatFx.toDouble()
                    + floatFy.toDouble() * floatFy.toDouble() + floatFz.toDouble() * floatFz.toDouble()
        )
        val factor = SensorManager.GRAVITY_EARTH / norm
        listener.onMeasurement(gravitySensorCollector, measurement)

        verify(exactly = 1) { gravityProcessorSpy.process(measurement) }
        verify(exactly = 1) {
            levelingProcessorSpy.process(
                floatFx.toDouble() * factor,
                floatFy.toDouble() * factor,
                floatFz.toDouble() * factor
            )
        }
        verify(exactly = 1) { attitudeSpy.fromQuaternion(levelingProcessor.attitude) }
        verify(exactly = 1) { coordinateTransformationSpy.fromRotation(attitudeSpy) }
        verify(exactly = 1) { attitudeSpy.toEulerAngles(eulerAngles) }

        val displayRoll = eulerAngles[0]
        val displayPitch = eulerAngles[1]

        verify(exactly = 1) {
            levelingAvailableListener.onLevelingAvailable(
                estimator,
                attitudeSpy,
                timestamp,
                displayRoll,
                displayPitch,
                coordinateTransformationSpy
            )
        }
    }

    @Test
    fun onAccelerometerAccuracyChanged_whenNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator2(context)

        val accelerometerSensorCollector: AccelerometerSensorCollector2? =
            getPrivateProperty(
                BaseLevelingEstimator2::class,
                estimator,
                "accelerometerSensorCollector"
            )
        requireNotNull(accelerometerSensorCollector)

        val listener = accelerometerSensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(accelerometerSensorCollector, SensorAccuracy.HIGH)
    }

    @Test
    fun onAccelerometerAccuracyChanged_whenListenerAvailable_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator2(
            context,
            useAccelerometer = true,
            accuracyChangedListener = accuracyChangedListener
        )

        val accelerometerSensorCollector: AccelerometerSensorCollector2? =
            getPrivateProperty(
                BaseLevelingEstimator2::class,
                estimator,
                "accelerometerSensorCollector"
            )
        requireNotNull(accelerometerSensorCollector)

        val listener = accelerometerSensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(accelerometerSensorCollector, SensorAccuracy.HIGH)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                estimator,
                SensorType.ACCELEROMETER_UNCALIBRATED,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun onAccelerometerEstimation_whenNoProcessedGravity_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator2(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            levelingAvailableListener = levelingAvailableListener
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spyk(attitude)
        setPrivateProperty(BaseLevelingEstimator2::class, estimator, "attitude", attitudeSpy)

        val eulerAngles: DoubleArray? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "eulerAngles")
        requireNotNull(eulerAngles)

        val coordinateTransformation: CoordinateTransformation? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "coordinateTransformation")
        requireNotNull(coordinateTransformation)
        assertEquals(FrameType.BODY_FRAME, coordinateTransformation.sourceType)
        assertEquals(
            FrameType.LOCAL_NAVIGATION_FRAME,
            coordinateTransformation.destinationType
        )
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        setPrivateProperty(
            BaseLevelingEstimator2::class,
            estimator,
            "coordinateTransformation",
            coordinateTransformationSpy
        )

        val randomizer = UniformRandomizer()
        val latitude =
            Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES))
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        // body attitude
        val roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val yaw1 = 0.0

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        val bodyC = CoordinateTransformation(
            roll1,
            pitch1,
            yaw1,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )

        // obtain specific force neglecting north component of gravity
        val cnb = bodyC.matrix
        val nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(latitude, height)
        val g = Matrix.newFromArray(nedGravity.asArray())
        g.multiplyByScalar(-1.0)
        val f = cnb.multiplyAndReturnNew(g)

        val fx = f.getElementAtIndex(0)
        val fy = f.getElementAtIndex(1)
        val fz = f.getElementAtIndex(2)

        val floatFx = fx.toFloat()
        val floatFy = fy.toFloat()
        val floatFz = fz.toFloat()

        val accelerometerSensorCollector: AccelerometerSensorCollector2? =
            getPrivateProperty(
                BaseLevelingEstimator2::class,
                estimator,
                "accelerometerSensorCollector"
            )
        requireNotNull(accelerometerSensorCollector)
        val listener = accelerometerSensorCollector.measurementListener
        requireNotNull(listener)

        val accelerometerGravityProcessor: AccelerometerGravityProcessor? =
            getPrivateProperty(
                BaseLevelingEstimator2::class,
                estimator,
                "accelerometerGravityProcessor"
            )
        requireNotNull(accelerometerGravityProcessor)
        val accelerometerGravityProcessorSpy = spyk(accelerometerGravityProcessor)
        setPrivateProperty(
            BaseLevelingEstimator2::class,
            estimator,
            "accelerometerGravityProcessor",
            accelerometerGravityProcessorSpy
        )

        val levelingProcessor: BaseLevelingProcessor? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "levelingProcessor")
        requireNotNull(levelingProcessor)
        val levelingProcessorSpy = spyk(levelingProcessor)
        estimator.setPrivateProperty("levelingProcessor", levelingProcessorSpy)

        val timestamp = SystemClock.elapsedRealtimeNanos()
        val measurement = AccelerometerSensorMeasurement(
            floatFy,
            floatFx,
            -floatFz,
            null,
            null,
            null,
            timestamp,
            SensorAccuracy.MEDIUM
        )
        listener.onMeasurement(accelerometerSensorCollector, measurement)

        verify(exactly = 1) { accelerometerGravityProcessorSpy.process(measurement) }
        verify { levelingProcessorSpy wasNot Called }
        verify { attitudeSpy wasNot Called }
        verify { coordinateTransformationSpy wasNot Called }
        verify { levelingAvailableListener wasNot Called }

        assertArrayEquals(eulerAngles, doubleArrayOf(0.0, 0.0, 0.0), 0.0)
    }

    @Test
    fun onAccelerometerEstimation_whenProcessedGravity_updatesAttitude() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator2(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spyk(attitude)
        setPrivateProperty(BaseLevelingEstimator2::class, estimator, "attitude", attitudeSpy)

        val eulerAngles: DoubleArray? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "eulerAngles")
        requireNotNull(eulerAngles)

        val coordinateTransformation: CoordinateTransformation? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "coordinateTransformation")
        requireNotNull(coordinateTransformation)
        assertEquals(FrameType.BODY_FRAME, coordinateTransformation.sourceType)
        assertEquals(
            FrameType.LOCAL_NAVIGATION_FRAME,
            coordinateTransformation.destinationType
        )
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        setPrivateProperty(
            BaseLevelingEstimator2::class,
            estimator,
            "coordinateTransformation",
            coordinateTransformationSpy
        )

        val randomizer = UniformRandomizer()
        val latitude =
            Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES))
        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        // body attitude
        val roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val yaw1 = 0.0

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        val bodyC = CoordinateTransformation(
            roll1,
            pitch1,
            yaw1,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )

        // obtain specific force neglecting north component of gravity
        val cnb = bodyC.matrix
        val nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(latitude, height)
        val g = Matrix.newFromArray(nedGravity.asArray())
        g.multiplyByScalar(-1.0)
        val f = cnb.multiplyAndReturnNew(g)

        val fx = f.getElementAtIndex(0)
        val fy = f.getElementAtIndex(1)
        val fz = f.getElementAtIndex(2)

        val floatFx = fx.toFloat()
        val floatFy = fy.toFloat()
        val floatFz = fz.toFloat()

        val accelerometerSensorCollector: AccelerometerSensorCollector2? =
            getPrivateProperty(
                BaseLevelingEstimator2::class,
                estimator,
                "accelerometerSensorCollector"
            )
        requireNotNull(accelerometerSensorCollector)
        val listener = accelerometerSensorCollector.measurementListener
        requireNotNull(listener)

        val accelerometerGravityProcessor: AccelerometerGravityProcessor? =
            getPrivateProperty(
                BaseLevelingEstimator2::class,
                estimator,
                "accelerometerGravityProcessor"
            )
        requireNotNull(accelerometerGravityProcessor)
        val accelerometerGravityProcessorSpy = spyk(accelerometerGravityProcessor)
        every { accelerometerGravityProcessorSpy.process(any(), any()) }.returns(true)
        every { accelerometerGravityProcessorSpy.gx }.returns(floatFx.toDouble())
        every { accelerometerGravityProcessorSpy.gy }.returns(floatFy.toDouble())
        every { accelerometerGravityProcessorSpy.gz }.returns(floatFz.toDouble())
        setPrivateProperty(
            BaseLevelingEstimator2::class,
            estimator,
            "accelerometerGravityProcessor",
            accelerometerGravityProcessorSpy
        )

        val levelingProcessor: BaseLevelingProcessor? =
            getPrivateProperty(BaseLevelingEstimator2::class, estimator, "levelingProcessor")
        requireNotNull(levelingProcessor)
        val levelingProcessorSpy = spyk(levelingProcessor)
        estimator.setPrivateProperty("levelingProcessor", levelingProcessorSpy)

        val timestamp = SystemClock.elapsedRealtimeNanos()
        val measurement = AccelerometerSensorMeasurement(
            floatFy,
            floatFx,
            -floatFz,
            null,
            null,
            null,
            timestamp,
            SensorAccuracy.MEDIUM
        )
        listener.onMeasurement(accelerometerSensorCollector, measurement)

        verify(exactly = 1) { accelerometerGravityProcessorSpy.process(measurement) }
        verify(exactly = 1) {
            levelingProcessorSpy.process(
                floatFx.toDouble(),
                floatFy.toDouble(),
                floatFz.toDouble()
            )
        }
        verify(exactly = 1) { attitudeSpy.fromQuaternion(levelingProcessor.attitude) }

        val attitude2 = Quaternion()
        bodyC.asRotation(attitude2)
        attitude2.normalize()
        attitude2.inverse()
        attitude2.normalize()
        assertTrue(attitudeSpy.equals(attitude2, ABSOLUTE_ERROR))

        verify { coordinateTransformationSpy wasNot Called }

        assertArrayEquals(eulerAngles, doubleArrayOf(0.0, 0.0, 0.0), 0.0)
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
        const val ABSOLUTE_ERROR = 1e-1

        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 4000.0

        const val MIN_ANGLE_DEGREES = -45.0
        const val MAX_ANGLE_DEGREES = 45.0
    }
}