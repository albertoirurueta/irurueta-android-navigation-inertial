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
package com.irurueta.android.navigation.inertial.estimators.attitude

import android.content.Context
import android.os.SystemClock
import android.view.Display
import android.view.Surface
import androidx.test.core.app.ApplicationProvider
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GravitySensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.filter.MeanAveragingFilter
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator
import com.irurueta.statistics.UniformRandomizer
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class LevelingEstimatorTest {

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator(context)

        // check
        assertSame(context, estimator.context)
        assertEquals(SensorDelay.GAME, estimator.sensorDelay)
        assertFalse(estimator.useAccelerometer)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
            estimator.accelerometerSensorType
        )
        assertNotNull(estimator.accelerometerAveragingFilter)
        assertFalse(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateEulerAngles)
        assertNull(estimator.levelingAvailableListener)
        assertNull(estimator.gravityEstimationListener)
        assertNull(estimator.accelerometerMeasurementListener)
        assertNull(estimator.gravityMeasurementListener)
        assertFalse(estimator.running)
    }

    @Test
    fun constructor_whenAllProperties_setsExpectedValues() {
        val levelingAvailableListener = mockk<LevelingEstimator.OnLevelingAvailableListener>()
        val accelerometerAveragingFilter = MeanAveragingFilter()
        val accelerometerMeasurementListener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>()
        val gravityMeasurementListener = mockk<GravitySensorCollector.OnMeasurementListener>()
        val gravityEstimationListener = mockk<GravityEstimator.OnEstimationListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator(
            context,
            SensorDelay.NORMAL,
            useAccelerometer = true,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            accelerometerAveragingFilter,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = false,
            levelingAvailableListener,
            gravityEstimationListener,
            accelerometerMeasurementListener,
            gravityMeasurementListener
        )

        // check
        assertSame(context, estimator.context)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertTrue(estimator.useAccelerometer)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.accelerometerSensorType
        )
        assertSame(accelerometerAveragingFilter, estimator.accelerometerAveragingFilter)
        assertTrue(estimator.estimateCoordinateTransformation)
        assertFalse(estimator.estimateEulerAngles)
        assertSame(levelingAvailableListener, estimator.levelingAvailableListener)
        assertSame(gravityEstimationListener, estimator.gravityEstimationListener)
        assertSame(accelerometerMeasurementListener, estimator.accelerometerMeasurementListener)
        assertSame(gravityMeasurementListener, estimator.gravityMeasurementListener)
        assertFalse(estimator.running)
    }

    @Test
    fun levelingAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator(context)

        // check default value
        assertNull(estimator.levelingAvailableListener)

        // set new value
        val listener = mockk<LevelingEstimator.OnLevelingAvailableListener>()
        estimator.levelingAvailableListener = listener

        // check
        assertSame(listener, estimator.levelingAvailableListener)
    }

    @Test
    fun gravityEstimationListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator(context)

        // check default value
        assertNull(estimator.gravityEstimationListener)

        // set new value
        val listener = mockk<GravityEstimator.OnEstimationListener>()
        estimator.gravityEstimationListener = listener
    }

    @Test
    fun accelerometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator(context)

        // check default value
        assertNull(estimator.accelerometerMeasurementListener)

        // set new value
        val listener = mockk<AccelerometerSensorCollector.OnMeasurementListener>()
        estimator.accelerometerMeasurementListener = listener

        // check
        assertSame(listener, estimator.accelerometerMeasurementListener)
    }

    @Test
    fun gravityMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator(context)

        // check default value
        assertNull(estimator.gravityMeasurementListener)

        // set new value
        val listener = mockk<GravitySensorCollector.OnMeasurementListener>()
        estimator.gravityMeasurementListener = listener

        // check
        assertSame(listener, estimator.gravityMeasurementListener)
    }

    @Test
    fun running_getsGravityEstimatorValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator(context)

        val gravityEstimator: GravityEstimator? =
            estimator.getPrivateProperty("gravityEstimator")
        requireNotNull(gravityEstimator)
        val gravityEstimatorSpy = spyk(gravityEstimator)
        every { gravityEstimatorSpy.running }.returns(true)
        estimator.setPrivateProperty("gravityEstimator", gravityEstimatorSpy)

        assertTrue(estimator.running)
        verify(exactly = 1) { gravityEstimatorSpy.running }
    }

    @Test
    fun start_startsGravityEstimator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator(context)

        val gravityEstimator: GravityEstimator? =
            estimator.getPrivateProperty("gravityEstimator")
        requireNotNull(gravityEstimator)
        val gravityEstimatorSpy = spyk(gravityEstimator)
        every { gravityEstimatorSpy.start() }.returns(true)
        estimator.setPrivateProperty("gravityEstimator", gravityEstimatorSpy)

        assertTrue(estimator.start())
        verify(exactly = 1) { gravityEstimatorSpy.start() }
    }

    @Test
    fun stop_stopsGravityEstimator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LevelingEstimator(context)

        val gravityEstimator: GravityEstimator? =
            estimator.getPrivateProperty("gravityEstimator")
        requireNotNull(gravityEstimator)
        val gravityEstimatorSpy = spyk(gravityEstimator)
        justRun { gravityEstimatorSpy.stop() }
        estimator.setPrivateProperty("gravityEstimator", gravityEstimatorSpy)

        estimator.stop()
        verify(exactly = 1) { gravityEstimatorSpy.stop() }
    }

    @Test
    fun onGravityEstimation_whenNoListenerNotEstimateCoordinateTransformationAndNotEstimateEulerAngles_updatesAttitude() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        val estimator = LevelingEstimator(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseLevelingEstimator::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spyk(attitude)
        setPrivateProperty(BaseLevelingEstimator::class, estimator, "attitude", attitudeSpy)

        val eulerAngles: DoubleArray? =
            getPrivateProperty(BaseLevelingEstimator::class, estimator, "eulerAngles")
        requireNotNull(eulerAngles)

        val coordinateTransformation: CoordinateTransformation? =
            getPrivateProperty(BaseLevelingEstimator::class, estimator, "coordinateTransformation")
        requireNotNull(coordinateTransformation)
        assertEquals(FrameType.BODY_FRAME, coordinateTransformation.sourceType)
        assertEquals(
            FrameType.LOCAL_NAVIGATION_FRAME,
            coordinateTransformation.destinationType
        )
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        estimator.setPrivateProperty("coordinateTransformation", coordinateTransformationSpy)

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

        val gravityEstimator: GravityEstimator? = estimator.getPrivateProperty("gravityEstimator")
        requireNotNull(gravityEstimator)
        val gravityEstimatorListener: GravityEstimator.OnEstimationListener? =
            gravityEstimator.estimationListener
        requireNotNull(gravityEstimatorListener)

        val timestamp = SystemClock.elapsedRealtimeNanos()
        gravityEstimatorListener.onEstimation(gravityEstimator, fx, fy, fz, timestamp)

        val expectedRoll =
            com.irurueta.navigation.inertial.estimators.LevelingEstimator.getRoll(fy, fz)
        val expectedPitch =
            com.irurueta.navigation.inertial.estimators.LevelingEstimator.getPitch(fx, fy, fz)

        verify(exactly = 1) { attitudeSpy.setFromEulerAngles(expectedRoll, expectedPitch, 0.0) }

        val attitude2 = Quaternion()
        bodyC.asRotation(attitude2)
        attitude2.normalize()
        attitude2.inverse()
        attitude2.normalize()
        assertTrue(attitudeSpy.equals(attitude2, 5.0))

        verify { coordinateTransformationSpy wasNot Called }

        assertArrayEquals(eulerAngles, doubleArrayOf(0.0, 0.0, 0.0), 0.0)
    }

    @Test
    fun onGravityEstimation_whenListenerNotEstimateCoordinateTransformationAndNotEstimateEulerAngles_updatesAttitude() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        val levelingAvailableListener =
            mockk<LevelingEstimator.OnLevelingAvailableListener>(relaxUnitFun = true)
        val gravityEstimationListener =
            mockk<GravityEstimator.OnEstimationListener>(relaxUnitFun = true)
        val estimator = LevelingEstimator(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            levelingAvailableListener = levelingAvailableListener,
            gravityEstimationListener = gravityEstimationListener
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseLevelingEstimator::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spyk(attitude)
        setPrivateProperty(BaseLevelingEstimator::class, estimator, "attitude", attitudeSpy)

        val eulerAngles: DoubleArray? =
            getPrivateProperty(BaseLevelingEstimator::class, estimator, "eulerAngles")
        requireNotNull(eulerAngles)

        val coordinateTransformation: CoordinateTransformation? =
            getPrivateProperty(BaseLevelingEstimator::class, estimator, "coordinateTransformation")
        requireNotNull(coordinateTransformation)
        assertEquals(FrameType.BODY_FRAME, coordinateTransformation.sourceType)
        assertEquals(
            FrameType.LOCAL_NAVIGATION_FRAME,
            coordinateTransformation.destinationType
        )
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        setPrivateProperty(
            BaseLevelingEstimator::class,
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

        val gravityEstimator: GravityEstimator? = estimator.getPrivateProperty("gravityEstimator")
        requireNotNull(gravityEstimator)
        val gravityEstimatorListener: GravityEstimator.OnEstimationListener? =
            gravityEstimator.estimationListener
        requireNotNull(gravityEstimatorListener)

        val timestamp = SystemClock.elapsedRealtimeNanos()
        gravityEstimatorListener.onEstimation(gravityEstimator, fx, fy, fz, timestamp)

        val expectedRoll =
            com.irurueta.navigation.inertial.estimators.LevelingEstimator.getRoll(fy, fz)
        val expectedPitch =
            com.irurueta.navigation.inertial.estimators.LevelingEstimator.getPitch(fx, fy, fz)

        verify(exactly = 1) { attitudeSpy.setFromEulerAngles(expectedRoll, expectedPitch, 0.0) }

        verify { coordinateTransformationSpy wasNot Called }

        assertArrayEquals(eulerAngles, doubleArrayOf(0.0, 0.0, 0.0), 0.0)

        verify(exactly = 1) {
            gravityEstimationListener.onEstimation(
                gravityEstimator,
                fx,
                fy,
                fz,
                timestamp
            )
        }
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
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        val levelingAvailableListener =
            mockk<LevelingEstimator.OnLevelingAvailableListener>(relaxUnitFun = true)

        val estimator = LevelingEstimator(
            context,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = true,
            levelingAvailableListener = levelingAvailableListener
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseLevelingEstimator::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spyk(attitude)
        setPrivateProperty(BaseLevelingEstimator::class, estimator, "attitude", attitudeSpy)

        val eulerAngles: DoubleArray? =
            getPrivateProperty(BaseLevelingEstimator::class, estimator, "eulerAngles")
        requireNotNull(eulerAngles)

        val coordinateTransformation: CoordinateTransformation? =
            getPrivateProperty(BaseLevelingEstimator::class, estimator, "coordinateTransformation")
        requireNotNull(coordinateTransformation)
        assertEquals(FrameType.BODY_FRAME, coordinateTransformation.sourceType)
        assertEquals(
            FrameType.LOCAL_NAVIGATION_FRAME,
            coordinateTransformation.destinationType
        )
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        setPrivateProperty(
            BaseLevelingEstimator::class,
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

        val gravityEstimator: GravityEstimator? = estimator.getPrivateProperty("gravityEstimator")
        requireNotNull(gravityEstimator)
        val gravityEstimatorListener: GravityEstimator.OnEstimationListener? =
            gravityEstimator.estimationListener
        requireNotNull(gravityEstimatorListener)

        val timestamp = SystemClock.elapsedRealtimeNanos()
        gravityEstimatorListener.onEstimation(gravityEstimator, fx, fy, fz, timestamp)

        val expectedRoll =
            com.irurueta.navigation.inertial.estimators.LevelingEstimator.getRoll(fy, fz)
        val expectedPitch =
            com.irurueta.navigation.inertial.estimators.LevelingEstimator.getPitch(fx, fy, fz)

        verify(exactly = 1) { attitudeSpy.setFromEulerAngles(expectedRoll, expectedPitch, 0.0) }
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
    fun onGravityEstimation_whenListenerIgnoreDisplayOrientation_updatesAttitude() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        val levelingAvailableListener =
            mockk<LevelingEstimator.OnLevelingAvailableListener>(relaxUnitFun = true)

        val estimator = LevelingEstimator(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            levelingAvailableListener = levelingAvailableListener
        )

        val attitude: Quaternion? =
            getPrivateProperty(BaseLevelingEstimator::class, estimator, "attitude")
        requireNotNull(attitude)
        val attitudeSpy = spyk(attitude)
        setPrivateProperty(BaseLevelingEstimator::class, estimator, "attitude", attitudeSpy)

        val eulerAngles: DoubleArray? =
            getPrivateProperty(BaseLevelingEstimator::class, estimator, "eulerAngles")
        requireNotNull(eulerAngles)

        val coordinateTransformation: CoordinateTransformation? =
            getPrivateProperty(BaseLevelingEstimator::class, estimator, "coordinateTransformation")
        requireNotNull(coordinateTransformation)
        assertEquals(FrameType.BODY_FRAME, coordinateTransformation.sourceType)
        assertEquals(
            FrameType.LOCAL_NAVIGATION_FRAME,
            coordinateTransformation.destinationType
        )
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        setPrivateProperty(
            BaseLevelingEstimator::class,
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

        val gravityEstimator: GravityEstimator? = estimator.getPrivateProperty("gravityEstimator")
        requireNotNull(gravityEstimator)
        val gravityEstimatorListener: GravityEstimator.OnEstimationListener? =
            gravityEstimator.estimationListener
        requireNotNull(gravityEstimatorListener)

        val timestamp = SystemClock.elapsedRealtimeNanos()
        gravityEstimatorListener.onEstimation(gravityEstimator, fx, fy, fz, timestamp)

        val expectedRoll =
            com.irurueta.navigation.inertial.estimators.LevelingEstimator.getRoll(fy, fz)
        val expectedPitch =
            com.irurueta.navigation.inertial.estimators.LevelingEstimator.getPitch(fx, fy, fz)

        verify(exactly = 1) { attitudeSpy.setFromEulerAngles(expectedRoll, expectedPitch, 0.0) }

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

    private companion object {
        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 4000.0

        const val MIN_ANGLE_DEGREES = -45.0
        const val MAX_ANGLE_DEGREES = 45.0
    }
}