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
import android.location.Location
import android.os.SystemClock
import android.view.Display
import android.view.Surface
import androidx.test.core.app.ApplicationProvider
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.old.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.old.collectors.GravitySensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.filter.MeanAveragingFilter
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator
import com.irurueta.statistics.UniformRandomizer
import io.mockk.Called
import io.mockk.every
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import io.mockk.justRun
import io.mockk.spyk
import io.mockk.verify
import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertSame
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class AccurateLevelingEstimatorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var levelingAvailableListener:
            AccurateLevelingEstimator.OnLevelingAvailableListener

    @MockK
    private lateinit var accelerometerMeasurementListener:
            AccelerometerSensorCollector.OnMeasurementListener

    @MockK
    private lateinit var gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener

    @MockK(relaxUnitFun = true)
    private lateinit var gravityEstimationListener: GravityEstimator.OnEstimationListener

    @MockK
    private lateinit var display: Display

    @MockK
    private lateinit var location: Location

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = AccurateLevelingEstimator(context, location)

        // check
        assertSame(context, estimator.context)
        assertSame(location, estimator.location)
        assertEquals(SensorDelay.GAME, estimator.sensorDelay)
        assertTrue(estimator.useAccelerometer)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.accelerometerSensorType
        )
        assertNotNull(estimator.accelerometerAveragingFilter)
        assertFalse(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateEulerAngles)
        assertNull(estimator.levelingAvailableListener)
        assertNull(estimator.gravityEstimationListener)
        assertFalse(estimator.running)
    }

    @Test
    fun constructor_whenAllProperties_setsExpectedValues() {
        val accelerometerAveragingFilter = MeanAveragingFilter()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = AccurateLevelingEstimator(
            context,
            location,
            SensorDelay.NORMAL,
            useAccelerometer = false,
            AccelerometerSensorType.ACCELEROMETER,
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
        assertSame(location, estimator.location)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertFalse(estimator.useAccelerometer)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
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
        val location = getLocation()
        val estimator = AccurateLevelingEstimator(context, location)

        // check default value
        assertNull(estimator.levelingAvailableListener)

        // set new value
        estimator.levelingAvailableListener = levelingAvailableListener

        // check
        assertSame(levelingAvailableListener, estimator.levelingAvailableListener)
    }

    @Test
    fun gravityEstimationListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = AccurateLevelingEstimator(context, location)

        // check default value
        assertNull(estimator.gravityEstimationListener)

        // set new value
        estimator.gravityEstimationListener = gravityEstimationListener
    }

    @Test
    fun accelerometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = AccurateLevelingEstimator(context, location)

        val gravityEstimator: GravityEstimator? =
            getPrivateProperty(BaseLevelingEstimator::class, estimator, "gravityEstimator")
        requireNotNull(gravityEstimator)

        // check default value
        assertNull(estimator.accelerometerMeasurementListener)
        assertNull(gravityEstimator.accelerometerMeasurementListener)

        // set new value
        estimator.accelerometerMeasurementListener = accelerometerMeasurementListener

        // check
        assertSame(accelerometerMeasurementListener, estimator.accelerometerMeasurementListener)
        assertSame(
            accelerometerMeasurementListener,
            gravityEstimator.accelerometerMeasurementListener
        )
    }

    @Test
    fun gravityMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = AccurateLevelingEstimator(context, location)

        // check default value
        assertNull(estimator.gravityMeasurementListener)

        // set new value
        estimator.gravityMeasurementListener = gravityMeasurementListener

        // check
        assertSame(gravityMeasurementListener, estimator.gravityMeasurementListener)
    }

    @Test
    fun running_getsGravityEstimatorValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = AccurateLevelingEstimator(context, location)

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
        val location = getLocation()
        val estimator = AccurateLevelingEstimator(context, location)

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
        val location = getLocation()
        val estimator = AccurateLevelingEstimator(context, location)

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
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        val location = getLocation()
        val estimator = AccurateLevelingEstimator(
            context,
            location,
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
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        setPrivateProperty(
            BaseLevelingEstimator::class,
            estimator,
            "coordinateTransformation",
            coordinateTransformationSpy
        )

        val randomizer = UniformRandomizer()
        val latitude = Math.toRadians(location.latitude)
        val height = location.altitude

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

        verify(exactly = 1) { attitudeSpy.setFromAxisAndRotation(any(), any()) }

        val attitude2 = Quaternion()
        bodyC.asRotation(attitude2)
        attitude2.inverse()
        attitude2.normalize()
        attitudeSpy.normalize()
        assertTrue(attitudeSpy.equals(attitude2, 2e-1))

        verify { coordinateTransformationSpy wasNot Called }

        assertArrayEquals(eulerAngles, doubleArrayOf(0.0, 0.0, 0.0), 0.0)
    }

    @Test
    fun onGravityEstimation_whenListenerNotEstimateCoordinateTransformationAndNotEstimateEulerAngles_updatesAttitude() {
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        val location = getLocation()
        val estimator = AccurateLevelingEstimator(
            context,
            location,
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
        val coordinateTransformationSpy = spyk(coordinateTransformation)
        setPrivateProperty(
            BaseLevelingEstimator::class,
            estimator,
            "coordinateTransformation",
            coordinateTransformationSpy
        )

        val randomizer = UniformRandomizer()
        val latitude = Math.toRadians(location.latitude)
        val height = location.altitude

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

        verify(exactly = 1) { attitudeSpy.setFromAxisAndRotation(any(), any()) }

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
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        val location = getLocation()
        val estimator = AccurateLevelingEstimator(
            context,
            location,
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
        val latitude = Math.toRadians(location.latitude)
        val height = location.altitude

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

        verify(exactly = 1) { attitudeSpy.setFromAxisAndRotation(any(), any()) }
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
    fun onGravityEstimation_whenListenerAndIgnoreDisplayOrientation_updatesAttitude() {
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        val location = getLocation()
        val estimator = AccurateLevelingEstimator(
            context,
            location,
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
        val latitude = Math.toRadians(location.latitude)
        val height = location.altitude

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
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
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

        verify(exactly = 1) { attitudeSpy.setFromAxisAndRotation(any(), any()) }

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
    }
}