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
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MeanAveragingFilter
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.statistics.UniformRandomizer
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class GravityEstimatorTest {

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityEstimator(context)

        // check
        assertSame(context, estimator.context)
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertFalse(estimator.useAccelerometer)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.accelerometerSensorType
        )
        assertNull(estimator.estimationListener)
        assertNotNull(estimator.accelerometerAveragingFilter)
        assertTrue(estimator.accelerometerAveragingFilter is LowPassAveragingFilter)
        assertNull(estimator.accelerometerMeasurementListener)
        assertNull(estimator.gravityMeasurementListener)
        assertFalse(estimator.running)
    }

    @Test
    fun constructor_whenAllProperties_setsExpectedValues() {
        val estimationListener = mockk<GravityEstimator.OnEstimationListener>()
        val accelerometerMeasurementListener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>()
        val gravityMeasurementListener = mockk<GravitySensorCollector.OnMeasurementListener>()
        val accelerometerAveragingFilter = MeanAveragingFilter()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityEstimator(
            context,
            SensorDelay.NORMAL,
            useAccelerometer = true,
            AccelerometerSensorType.ACCELEROMETER,
            estimationListener,
            accelerometerAveragingFilter,
            accelerometerMeasurementListener,
            gravityMeasurementListener
        )

        // check
        assertSame(context, estimator.context)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertTrue(estimator.useAccelerometer)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            estimator.accelerometerSensorType
        )
        assertSame(estimationListener, estimator.estimationListener)
        assertSame(accelerometerAveragingFilter, estimator.accelerometerAveragingFilter)
        assertSame(accelerometerMeasurementListener, estimator.accelerometerMeasurementListener)
        assertSame(gravityMeasurementListener, estimator.gravityMeasurementListener)
        assertFalse(estimator.running)
    }

    @Test
    fun estimationListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityEstimator(context)

        // check default value
        assertNull(estimator.estimationListener)

        // set new value
        val estimationListener = mockk<GravityEstimator.OnEstimationListener>()
        estimator.estimationListener = estimationListener

        // check
        assertSame(estimationListener, estimator.estimationListener)
    }

    @Test
    fun accelerometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityEstimator(context)

        // check default value
        assertNull(estimator.accelerometerMeasurementListener)

        // set new value
        val accelerometerMeasurementListener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>()
        estimator.accelerometerMeasurementListener = accelerometerMeasurementListener

        // check
        assertSame(accelerometerMeasurementListener, estimator.accelerometerMeasurementListener)
    }

    @Test
    fun gravityMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityEstimator(context)

        // check default value
        assertNull(estimator.gravityMeasurementListener)

        // set new value
        val gravityMeasurementListener = mockk<GravitySensorCollector.OnMeasurementListener>()
        estimator.gravityMeasurementListener = gravityMeasurementListener

        // check
        assertSame(gravityMeasurementListener, estimator.gravityMeasurementListener)
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityEstimator(context)

        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        estimator.start()
    }

    @Test
    fun start_whenNotRunningAndUseAccelerometer_resetsAndStartsAccelerometerSensorCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityEstimator(context, useAccelerometer = true)

        val accelerometerAveragingFilter: AveragingFilter? =
            estimator.getPrivateProperty("accelerometerAveragingFilter")
        requireNotNull(accelerometerAveragingFilter)
        val accelerometerAveragingFilterSpy = spyk(accelerometerAveragingFilter)
        estimator.setPrivateProperty(
            "accelerometerAveragingFilter",
            accelerometerAveragingFilterSpy
        )

        val gravitySensorCollector: GravitySensorCollector? =
            estimator.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        estimator.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        assertTrue(estimator.start())

        assertTrue(estimator.running)
        verify(exactly = 1) { accelerometerAveragingFilterSpy.reset() }
        verify(exactly = 1) { accelerometerSensorCollectorSpy.start() }
        verify { gravitySensorCollectorSpy wasNot Called }
    }

    @Test
    fun start_whenNotRunningAndNotUseAccelerometer_resetsAndStartsAccelerometerSensorCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityEstimator(context, useAccelerometer = false)

        val accelerometerAveragingFilter: AveragingFilter? =
            estimator.getPrivateProperty("accelerometerAveragingFilter")
        requireNotNull(accelerometerAveragingFilter)
        val accelerometerAveragingFilterSpy = spyk(accelerometerAveragingFilter)
        estimator.setPrivateProperty(
            "accelerometerAveragingFilter",
            accelerometerAveragingFilterSpy
        )

        val gravitySensorCollector: GravitySensorCollector? =
            estimator.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        every { gravitySensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        assertTrue(estimator.start())

        assertTrue(estimator.running)
        verify(exactly = 1) { accelerometerAveragingFilterSpy.reset() }
        verify { accelerometerSensorCollectorSpy wasNot Called }
        verify(exactly = 1) { gravitySensorCollectorSpy.start() }
    }

    @Test
    fun stop_stopsCollectors() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityEstimator(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        val gravitySensorCollector: GravitySensorCollector? =
            estimator.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        justRun { gravitySensorCollectorSpy.stop() }
        estimator.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        justRun { accelerometerSensorCollectorSpy.stop() }
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        estimator.stop()

        verify(exactly = 1) { gravitySensorCollectorSpy.stop() }
        verify(exactly = 1) { accelerometerSensorCollectorSpy.stop() }
        assertFalse(estimator.running)
    }

    @Test
    fun gravitySensorCollector_whenMeasurementAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityEstimator(context)

        val gravitySensorCollector: GravitySensorCollector? =
            estimator.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)

        val measurementListener = gravitySensorCollector.measurementListener
        requireNotNull(measurementListener)

        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val g = randomizer.nextDouble()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        measurementListener.onMeasurement(gx, gy, gz, g, timestamp, SensorAccuracy.HIGH)
    }

    @Test
    fun gravitySensorCollector_whenMeasurementAndListener_notifiesEstimation() {
        val estimationListener = mockk<GravityEstimator.OnEstimationListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityEstimator(context, estimationListener = estimationListener)

        val gravitySensorCollector: GravitySensorCollector? =
            estimator.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)

        val measurementListener = gravitySensorCollector.measurementListener
        requireNotNull(measurementListener)

        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val g = randomizer.nextDouble()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        measurementListener.onMeasurement(gx, gy, gz, g, timestamp, SensorAccuracy.HIGH)

        verify(exactly = 1) {
            estimationListener.onEstimation(
                estimator,
                gy.toDouble(),
                gx.toDouble(),
                -gz.toDouble(),
                timestamp
            )
        }
    }

    @Test
    fun gravitySensorCollector_whenMeasurementAndGravityListener_notifiesGravityMeasurement() {
        val gravityMeasurementListener =
            mockk<GravitySensorCollector.OnMeasurementListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator =
            GravityEstimator(context, gravityMeasurementListener = gravityMeasurementListener)

        val gravitySensorCollector: GravitySensorCollector? =
            estimator.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)

        val measurementListener = gravitySensorCollector.measurementListener
        requireNotNull(measurementListener)

        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val g = randomizer.nextDouble()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        measurementListener.onMeasurement(gx, gy, gz, g, timestamp, SensorAccuracy.HIGH)

        verify(exactly = 1) {
            gravityMeasurementListener.onMeasurement(gx, gy, gz, g, timestamp, SensorAccuracy.HIGH)
        }
    }

    @Test
    fun accelerometerSensorCollector_whenMeasurementAndNoListener_filters() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityEstimator(context)

        val accelerometerAveragingFilter: AveragingFilter? =
            estimator.getPrivateProperty("accelerometerAveragingFilter")
        requireNotNull(accelerometerAveragingFilter)
        val accelerometerAveragingFilterSpy = spyk(accelerometerAveragingFilter)
        estimator.setPrivateProperty(
            "accelerometerAveragingFilter",
            accelerometerAveragingFilterSpy
        )

        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)

        val measurementListener = accelerometerSensorCollector.measurementListener
        requireNotNull(measurementListener)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        measurementListener.onMeasurement(ax, ay, az, bx, by, bz, timestamp, SensorAccuracy.HIGH)

        verify(exactly = 1) {
            accelerometerAveragingFilterSpy.filter(
                ax.toDouble() - bx.toDouble(),
                ay.toDouble() - by.toDouble(),
                az.toDouble() - bz.toDouble(),
                any(),
                timestamp
            )
        }
    }

    @Test
    fun accelerometerSensorCollector_whenMeasurementAndListener_notifiesEstimation() {
        val estimationListener = mockk<GravityEstimator.OnEstimationListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityEstimator(context, estimationListener = estimationListener)

        val accelerometerAveragingFilter: AveragingFilter? =
            estimator.getPrivateProperty("accelerometerAveragingFilter")
        requireNotNull(accelerometerAveragingFilter)
        assertTrue(accelerometerAveragingFilter is LowPassAveragingFilter)
        val accelerometerAveragingFilterSpy = spyk(accelerometerAveragingFilter)
        estimator.setPrivateProperty(
            "accelerometerAveragingFilter",
            accelerometerAveragingFilterSpy
        )

        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)

        val measurementListener = accelerometerSensorCollector.measurementListener
        requireNotNull(measurementListener)

        // receive 1st measurement
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        measurementListener.onMeasurement(ax, ay, az, bx, by, bz, timestamp, SensorAccuracy.HIGH)

        verify(exactly = 1) {
            accelerometerAveragingFilterSpy.filter(
                ax.toDouble() - bx.toDouble(),
                ay.toDouble() - by.toDouble(),
                az.toDouble() - bz.toDouble(),
                any(),
                timestamp
            )
        }

        verify { estimationListener wasNot Called }

        // receive 2nd measurement
        measurementListener.onMeasurement(ax, ay, az, bx, by, bz, timestamp, SensorAccuracy.HIGH)

        val filterOutputList = mutableListOf<DoubleArray>()
        verify(exactly = 2) {
            accelerometerAveragingFilterSpy.filter(
                ax.toDouble() - bx.toDouble(),
                ay.toDouble() - by.toDouble(),
                az.toDouble() - bz.toDouble(),
                capture(filterOutputList),
                timestamp
            )
        }

        val filterOutput = filterOutputList[1]
        verify(exactly = 1) {
            estimationListener.onEstimation(
                estimator,
                filterOutput[1],
                filterOutput[0],
                -filterOutput[2],
                timestamp
            )
        }
    }

    @Test
    fun accelerometerSensorCollector_whenMeasurementAndMeasurementListener_notifiesEstimation() {
        val accelerometerMeasurementListener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = GravityEstimator(
            context,
            accelerometerMeasurementListener = accelerometerMeasurementListener
        )

        val accelerometerAveragingFilter: AveragingFilter? =
            estimator.getPrivateProperty("accelerometerAveragingFilter")
        requireNotNull(accelerometerAveragingFilter)
        assertTrue(accelerometerAveragingFilter is LowPassAveragingFilter)
        val accelerometerAveragingFilterSpy = spyk(accelerometerAveragingFilter)
        estimator.setPrivateProperty(
            "accelerometerAveragingFilter",
            accelerometerAveragingFilterSpy
        )

        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)

        val measurementListener = accelerometerSensorCollector.measurementListener
        requireNotNull(measurementListener)

        // receive 1st measurement
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        measurementListener.onMeasurement(ax, ay, az, bx, by, bz, timestamp, SensorAccuracy.HIGH)

        verify(exactly = 1) {
            accelerometerAveragingFilterSpy.filter(
                ax.toDouble() - bx.toDouble(),
                ay.toDouble() - by.toDouble(),
                az.toDouble() - bz.toDouble(),
                any(),
                timestamp
            )
        }

        verify(exactly = 1) {
            accelerometerMeasurementListener.onMeasurement(
                ax,
                ay,
                az,
                bx,
                by,
                bz,
                timestamp,
                SensorAccuracy.HIGH
            )
        }

        // receive 2nd measurement
        measurementListener.onMeasurement(ax, ay, az, bx, by, bz, timestamp, SensorAccuracy.HIGH)

        val filterOutputList = mutableListOf<DoubleArray>()
        verify(exactly = 2) {
            accelerometerAveragingFilterSpy.filter(
                ax.toDouble() - bx.toDouble(),
                ay.toDouble() - by.toDouble(),
                az.toDouble() - bz.toDouble(),
                capture(filterOutputList),
                timestamp
            )
        }

        verify(exactly = 2) {
            accelerometerMeasurementListener.onMeasurement(
                ax,
                ay,
                az,
                bx,
                by,
                bz,
                timestamp,
                SensorAccuracy.HIGH
            )
        }
    }
}