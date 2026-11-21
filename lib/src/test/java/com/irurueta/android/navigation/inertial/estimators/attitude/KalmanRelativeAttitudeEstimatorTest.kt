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
import android.location.Location
import android.os.SystemClock
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.collectors.AccelerometerAndGyroscopeSyncedSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerAndGyroscopeSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorType
import com.irurueta.android.navigation.inertial.processors.attitude.KalmanRelativeAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.filters.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.processors.filters.MeanAveragingFilter
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegrator
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegratorType
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import io.mockk.Called
import io.mockk.every
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import io.mockk.justRun
import io.mockk.mockkStatic
import io.mockk.slot
import io.mockk.spyk
import io.mockk.verify
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test

class KalmanRelativeAttitudeEstimatorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK
    private lateinit var location: Location

    @MockK
    private lateinit var context: Context

    @MockK
    private lateinit var attitudeAvailableListener: KalmanRelativeAttitudeEstimator.OnAttitudeAvailableListener

    @MockK
    private lateinit var accuracyChangedListener: KalmanRelativeAttitudeEstimator.OnAccuracyChangedListener

    @Test
    fun constructor_whenDefaultValues_setsExpectedValues() {
        val estimator = KalmanRelativeAttitudeEstimator(context)

        assertSame(context, estimator.context)
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            estimator.gyroscopeSensorType
        )
        assertTrue(estimator.averagingFilter is LowPassAveragingFilter)
        assertNull(estimator.location)
        assertTrue(estimator.adjustGravityNorm)
        assertFalse(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateEulerAngles)
        assertTrue(estimator.estimateCovariances)
        assertEquals(
            KalmanRelativeAttitudeProcessor.DEFAULT_GYROSCOPE_NOISE_PSD,
            estimator.gyroscopeNoisePsd,
            0.0
        )
        assertEquals(
            KalmanRelativeAttitudeProcessor.DEFAULT_ACCELEROMETER_NOISE_STANDARD_DEVIATION,
            estimator.accelerometerNoiseStandardDeviation,
            0.0
        )
        assertEquals(
            QuaternionStepIntegrator.DEFAULT_TYPE,
            estimator.quaternionStepIntegratorType
        )
        assertEquals(
            KalmanRelativeAttitudeProcessor.DEFAULT_FREE_FALL_THRESHOLD,
            estimator.freeFallThreshold,
            0.0
        )
        assertEquals(
            KalmanRelativeAttitudeProcessor.DEFAULT_SYMMETRIC_THRESHOLD,
            estimator.symmetricThreshold,
            0.0
        )
        assertEquals(
            KalmanRelativeAttitudeEstimator.DEFAULT_MINIMUM_UNRELIABLE_DURATION_SECONDS,
            estimator.minimumUnreliableDurationSeconds,
            0.0
        )
        assertNull(estimator.attitudeAvailableListener)
        assertNull(estimator.accuracyChangedListener)
    }

    @Test
    fun constructor_whenAllValues_setsExpectedValues() {
        val averagingFilter =
            MeanAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()
        val location = getLocation()
        val randomizer = UniformRandomizer()
        val gyroscopeNoisePsd = randomizer.nextDouble()
        val accelerometerNoiseStandardDeviation = randomizer.nextDouble()
        val freeFalThreshold = randomizer.nextDouble()
        val symmetricThreshold = randomizer.nextDouble()
        val minimumUnreliableDurationSeconds = randomizer.nextDouble()

        val estimator = KalmanRelativeAttitudeEstimator(
            context,
            SensorDelay.NORMAL,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            averagingFilter,
            location,
            adjustGravityNorm = false,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = false,
            estimateCovariances = false,
            gyroscopeNoisePsd,
            accelerometerNoiseStandardDeviation,
            QuaternionStepIntegratorType.EULER_METHOD,
            freeFalThreshold,
            symmetricThreshold,
            minimumUnreliableDurationSeconds,
            attitudeAvailableListener,
            accuracyChangedListener
        )

        assertSame(context, estimator.context)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            estimator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            estimator.gyroscopeSensorType
        )
        assertSame(averagingFilter, estimator.averagingFilter)
        assertSame(location, estimator.location)
        assertFalse(estimator.adjustGravityNorm)
        assertTrue(estimator.estimateCoordinateTransformation)
        assertFalse(estimator.estimateEulerAngles)
        assertFalse(estimator.estimateCovariances)
        assertEquals(
            gyroscopeNoisePsd,
            estimator.gyroscopeNoisePsd,
            0.0
        )
        assertEquals(
            accelerometerNoiseStandardDeviation,
            estimator.accelerometerNoiseStandardDeviation,
            0.0
        )
        assertEquals(
            QuaternionStepIntegratorType.EULER_METHOD,
            estimator.quaternionStepIntegratorType
        )
        assertEquals(
            freeFalThreshold,
            estimator.freeFallThreshold,
            0.0
        )
        assertEquals(
            symmetricThreshold,
            estimator.symmetricThreshold,
            0.0
        )
        assertEquals(
            minimumUnreliableDurationSeconds,
            estimator.minimumUnreliableDurationSeconds,
            0.0
        )
        assertSame(
            attitudeAvailableListener,
            estimator.attitudeAvailableListener
        )
        assertSame(accuracyChangedListener, estimator.accuracyChangedListener)
    }

    @Test
    fun location_setsExpectedValue() {
        val estimator = KalmanRelativeAttitudeEstimator(context)

        // check default value
        assertNull(estimator.location)

        val processor: KalmanRelativeAttitudeProcessor? = estimator.getPrivateProperty(
            "processor"
        )
        requireNotNull(processor)
        assertNull(processor.location)

        // set new value
        val location = getLocation()
        estimator.location = location

        // check
        assertSame(location, estimator.location)
        assertSame(location, processor.location)
    }

    @Test
    fun adjustGravityNorm_setsExpectedValue() {
        val estimator = KalmanRelativeAttitudeEstimator(context)

        // check default value
        assertTrue(estimator.adjustGravityNorm)

        val processor: KalmanRelativeAttitudeProcessor? = estimator.getPrivateProperty(
            "processor"
        )
        requireNotNull(processor)
        assertTrue(processor.adjustGravityNorm)


        // set new value
        estimator.adjustGravityNorm = false

        // check
        assertFalse(estimator.adjustGravityNorm)
        assertFalse(processor.adjustGravityNorm)
    }

    @Test
    fun processCovariance_whenNoEstimateEulerAnglesAndNoEstimateCovariances_returnsFalse() {
        val estimator = KalmanRelativeAttitudeEstimator(
            context,
            estimateEulerAngles = false,
            estimateCovariances = false
        )

        val processCovariance: Boolean? = estimator.getPrivateProperty("processCovariance")
        requireNotNull(processCovariance)
        assertFalse(processCovariance)
    }

    @Test
    fun processCovariance_whenEstimateEulerAngles_returnsTrue() {
        val estimator = KalmanRelativeAttitudeEstimator(
            context,
            estimateEulerAngles = true,
            estimateCovariances = false
        )

        val processCovariance: Boolean? = estimator.getPrivateProperty("processCovariance")
        requireNotNull(processCovariance)
        assertTrue(processCovariance)
    }

    @Test
    fun processCovariance_whenEstimateCovariances_returnsTrue() {
        val estimator = KalmanRelativeAttitudeEstimator(
            context,
            estimateEulerAngles = false,
            estimateCovariances = true
        )

        val processCovariance: Boolean? = estimator.getPrivateProperty("processCovariance")
        requireNotNull(processCovariance)
        assertTrue(processCovariance)
    }

    @Test
    fun processCovariance_whenEstimateEulerAnglesAndEstimateCovariances_returnsTrue() {
        val estimator = KalmanRelativeAttitudeEstimator(
            context,
            estimateEulerAngles = true,
            estimateCovariances = true
        )

        val processCovariance: Boolean? = estimator.getPrivateProperty("processCovariance")
        requireNotNull(processCovariance)
        assertTrue(processCovariance)
    }

    @Test
    fun processEulerAnglesCovariance_whenNoEstimateEulerAnglesAndNoEstimateCovariances_returnsFalse() {
        val estimator = KalmanRelativeAttitudeEstimator(
            context,
            estimateEulerAngles = false,
            estimateCovariances = false
        )

        val processEulerAnglesCovariance: Boolean? =
            estimator.getPrivateProperty("processEulerAnglesCovariance")
        requireNotNull(processEulerAnglesCovariance)
        assertFalse(processEulerAnglesCovariance)
    }

    @Test
    fun processEulerAnglesCovariance_whenEstimateEulerAngles_returnsFalse() {
        val estimator = KalmanRelativeAttitudeEstimator(
            context,
            estimateEulerAngles = true,
            estimateCovariances = false
        )

        val processEulerAnglesCovariance: Boolean? =
            estimator.getPrivateProperty("processEulerAnglesCovariance")
        requireNotNull(processEulerAnglesCovariance)
        assertFalse(processEulerAnglesCovariance)
    }

    @Test
    fun processEulerAnglesCovariance_whenEstimateCovariances_returnsFalse() {
        val estimator = KalmanRelativeAttitudeEstimator(
            context,
            estimateEulerAngles = false,
            estimateCovariances = true
        )

        val processEulerAnglesCovariance: Boolean? =
            estimator.getPrivateProperty("processEulerAnglesCovariance")
        requireNotNull(processEulerAnglesCovariance)
        assertFalse(processEulerAnglesCovariance)
    }

    @Test
    fun processEulerAnglesCovariance_whenEstimateEulerAnglesAndEstimateCovariances_returnsTrue() {
        val estimator = KalmanRelativeAttitudeEstimator(
            context,
            estimateEulerAngles = true,
            estimateCovariances = true
        )

        val processEulerAnglesCovariance: Boolean? =
            estimator.getPrivateProperty("processEulerAnglesCovariance")
        requireNotNull(processEulerAnglesCovariance)
        assertTrue(processEulerAnglesCovariance)
    }

    @Test
    fun processor_whenNumericalInstabilitiesListenerAndNoListener_makesEstimatorUnreliable() {
        val estimator = KalmanRelativeAttitudeEstimator(context)

        val processor: KalmanRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("processor")
        requireNotNull(processor)

        // check default value
        val unreliable1: Boolean? = estimator.getPrivateProperty("unreliable")
        requireNotNull(unreliable1)
        assertFalse(unreliable1)

        val lastTimestamp1: Long? = estimator.getPrivateProperty("lastTimestamp")
        requireNotNull(lastTimestamp1)
        assertEquals(-1L, lastTimestamp1)

        val unreliableTimestamp1: Long? = estimator.getPrivateProperty("unreliableTimestamp")
        requireNotNull(unreliableTimestamp1)
        assertEquals(-1L, unreliableTimestamp1)

        // set new lastTimestamp
        estimator.setPrivateProperty("lastTimestamp", 2L)

        val numericalInstabilitiesListener = processor.numericalInstabilitiesListener
        requireNotNull(numericalInstabilitiesListener)
        numericalInstabilitiesListener.onNumericalInstabilityDetected(processor)

        val unreliable2: Boolean? = estimator.getPrivateProperty("unreliable")
        requireNotNull(unreliable2)
        assertTrue(unreliable2)

        val lastTimestamp2: Long? = estimator.getPrivateProperty("lastTimestamp")
        requireNotNull(lastTimestamp2)
        assertEquals(2L, lastTimestamp2)

        val unreliableTimestamp2: Long? = estimator.getPrivateProperty("unreliableTimestamp")
        requireNotNull(unreliableTimestamp2)
        assertEquals(lastTimestamp2, unreliableTimestamp2)
    }

    @Test
    fun processor_whenNumericalInstabilitiesListenerAndListener_makesEstimatorUnreliable() {
        justRun { accuracyChangedListener.onAccuracyChanged(any(), any(), any()) }

        val estimator = KalmanRelativeAttitudeEstimator(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val processor: KalmanRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("processor")
        requireNotNull(processor)

        // check default value
        val unreliable1: Boolean? = estimator.getPrivateProperty("unreliable")
        requireNotNull(unreliable1)
        assertFalse(unreliable1)

        val lastTimestamp1: Long? = estimator.getPrivateProperty("lastTimestamp")
        requireNotNull(lastTimestamp1)
        assertEquals(-1L, lastTimestamp1)

        val unreliableTimestamp1: Long? = estimator.getPrivateProperty("unreliableTimestamp")
        requireNotNull(unreliableTimestamp1)
        assertEquals(-1L, unreliableTimestamp1)

        // set new lastTimestamp
        estimator.setPrivateProperty("lastTimestamp", 2L)

        val numericalInstabilitiesListener = processor.numericalInstabilitiesListener
        requireNotNull(numericalInstabilitiesListener)
        numericalInstabilitiesListener.onNumericalInstabilityDetected(processor)

        val unreliable2: Boolean? = estimator.getPrivateProperty("unreliable")
        requireNotNull(unreliable2)
        assertTrue(unreliable2)

        val lastTimestamp2: Long? = estimator.getPrivateProperty("lastTimestamp")
        requireNotNull(lastTimestamp2)
        assertEquals(2L, lastTimestamp2)

        val unreliableTimestamp2: Long? = estimator.getPrivateProperty("unreliableTimestamp")
        requireNotNull(unreliableTimestamp2)
        assertEquals(lastTimestamp2, unreliableTimestamp2)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                estimator, SensorType.RELATIVE_ATTITUDE,
                SensorAccuracy.UNRELIABLE
            )
        }
    }

    @Test
    fun collectorMeasurementListener_whenProcessReturnsFalse_makesNoAction() {
        val estimator = KalmanRelativeAttitudeEstimator(context)

        val processor: KalmanRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("processor")
        requireNotNull(processor)
        val processorSpy = spyk(processor)
        estimator.setPrivateProperty("processor", processorSpy)

        every { processorSpy.process(any()) }.returns(false)

        val collector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)

        val measurementListener = collector.measurementListener
        requireNotNull(measurementListener)

        // call listener
        val timestamp = System.nanoTime()
        val measurement = AccelerometerAndGyroscopeSyncedSensorMeasurement(timestamp = timestamp)
        measurementListener.onMeasurement(collector, measurement)

        verify(exactly = 0) { processorSpy.nedAttitude }
    }

    @Test
    fun collectorMeasurementListener_whenNoEstimateCoordinateTransformationAndNoListener_processesMeasurement() {
        val estimator = KalmanRelativeAttitudeEstimator(context)

        val processor: KalmanRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("processor")
        requireNotNull(processor)
        val processorSpy = spyk(processor)
        estimator.setPrivateProperty("processor", processorSpy)

        every { processorSpy.process(any()) }.returns(true)
        val randomizer = UniformRandomizer()
        val nedAttitude = Quaternion(
            randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES),
            randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES),
            randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        )
        every { processorSpy.nedAttitude }.returns(nedAttitude)


        val collector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)

        val measurementListener = collector.measurementListener
        requireNotNull(measurementListener)

        // call listener
        val timestamp = System.nanoTime()
        val measurement = AccelerometerAndGyroscopeSyncedSensorMeasurement(timestamp = timestamp)
        measurementListener.onMeasurement(collector, measurement)

        // check
        verify(exactly = 1) { processorSpy.process(measurement) }
        verify(exactly = 1) { processorSpy.nedAttitude }

        val attitude: Quaternion? = estimator.getPrivateProperty("attitude")
        requireNotNull(attitude)
        assertEquals(nedAttitude, attitude)

        val lastTimestamp: Long? = estimator.getPrivateProperty("lastTimestamp")
        requireNotNull(lastTimestamp)
        assertEquals(timestamp, lastTimestamp)
    }

    @Test
    fun collectorMeasurementListener_whenAllEstimationEnabledAndListener_processesMeasurement() {

        justRun {
            attitudeAvailableListener.onAttitudeAvailable(
                any(),
                any(),
                any(),
                any(),
                any(),
                any(),
                any(),
                any(),
                any()
            )
        }

        val estimator = KalmanRelativeAttitudeEstimator(
            context,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = true,
            estimateCovariances = true,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val processor: KalmanRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("processor")
        requireNotNull(processor)
        val processorSpy = spyk(processor)
        estimator.setPrivateProperty("processor", processorSpy)

        every { processorSpy.process(any()) }.returns(true)
        val randomizer = UniformRandomizer()
        val nedAttitude = Quaternion(
            randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES),
            randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES),
            randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        )
        every { processorSpy.nedAttitude }.returns(nedAttitude)


        val collector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)

        val measurementListener = collector.measurementListener
        requireNotNull(measurementListener)

        // call listener
        val timestamp = System.nanoTime()
        val measurement = AccelerometerAndGyroscopeSyncedSensorMeasurement(timestamp = timestamp)
        measurementListener.onMeasurement(collector, measurement)

        // check
        verify(exactly = 1) { processorSpy.process(measurement) }
        verify(exactly = 1) { processorSpy.nedAttitude }

        val attitude: Quaternion? = estimator.getPrivateProperty("attitude")
        requireNotNull(attitude)
        assertEquals(nedAttitude, attitude)

        val lastTimestamp: Long? = estimator.getPrivateProperty("lastTimestamp")
        requireNotNull(lastTimestamp)
        assertEquals(timestamp, lastTimestamp)

        val attitudeSlot = slot<Quaternion>()
        val rollSlot = slot<Double>()
        val pitchSlot = slot<Double>()
        val yawSlot = slot<Double>()
        val cSlot = slot<CoordinateTransformation>()
        val quaternionCovarianceSlot = slot<Matrix>()
        val eulerAnglesCovarianceSlot = slot<Matrix>()

        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                capture(attitudeSlot),
                timestamp,
                capture(rollSlot),
                capture(pitchSlot),
                capture(yawSlot),
                capture(cSlot),
                capture(quaternionCovarianceSlot),
                capture(eulerAnglesCovarianceSlot)
            )
        }

        val attitude2 = attitudeSlot.captured
        val roll = rollSlot.captured
        val pitch = pitchSlot.captured
        val yaw = yawSlot.captured
        val c = cSlot.captured
        val quaternionCovariance = quaternionCovarianceSlot.captured
        val eulerAnglesCovariance = eulerAnglesCovarianceSlot.captured

        assertEquals(nedAttitude, attitude2)
        assertEquals(0.0, roll, 0.0)
        assertEquals(0.0, pitch, 0.0)
        assertEquals(0.0, yaw, 0.0)
        assertEquals(
            CoordinateTransformation(
                attitude,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            ), c
        )
        assertEquals(Matrix(4, 4), quaternionCovariance)
        assertEquals(Matrix(3, 3), eulerAnglesCovariance)
    }

    @Test
    fun collectorMeasurementListener_whenAllEstimationDisabledAndListener_processesMeasurement() {

        justRun {
            attitudeAvailableListener.onAttitudeAvailable(
                any(),
                any(),
                any(),
                any(),
                any(),
                any(),
                any(),
                any(),
                any()
            )
        }

        val estimator = KalmanRelativeAttitudeEstimator(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            estimateCovariances = false,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val processor: KalmanRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("processor")
        requireNotNull(processor)
        val processorSpy = spyk(processor)
        estimator.setPrivateProperty("processor", processorSpy)

        every { processorSpy.process(any()) }.returns(true)
        val randomizer = UniformRandomizer()
        val nedAttitude = Quaternion(
            randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES),
            randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES),
            randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        )
        every { processorSpy.nedAttitude }.returns(nedAttitude)

        val collector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)

        val measurementListener = collector.measurementListener
        requireNotNull(measurementListener)

        // call listener
        val timestamp = System.nanoTime()
        val measurement = AccelerometerAndGyroscopeSyncedSensorMeasurement(timestamp = timestamp)
        measurementListener.onMeasurement(collector, measurement)

        // check
        verify(exactly = 1) { processorSpy.process(measurement) }
        verify(exactly = 1) { processorSpy.nedAttitude }

        val attitude: Quaternion? = estimator.getPrivateProperty("attitude")
        requireNotNull(attitude)
        assertEquals(nedAttitude, attitude)

        val lastTimestamp: Long? = estimator.getPrivateProperty("lastTimestamp")
        requireNotNull(lastTimestamp)
        assertEquals(timestamp, lastTimestamp)

        val attitudeSlot = slot<Quaternion>()
        val rollSlot = slot<Double>()
        val pitchSlot = slot<Double>()
        val yawSlot = slot<Double>()
        val cSlot = slot<CoordinateTransformation>()
        val quaternionCovarianceSlot = slot<Matrix>()
        val eulerAnglesCovarianceSlot = slot<Matrix>()

        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                capture(attitudeSlot),
                timestamp,
                capture(rollSlot),
                capture(pitchSlot),
                capture(yawSlot),
                capture(cSlot),
                capture(quaternionCovarianceSlot),
                capture(eulerAnglesCovarianceSlot)
            )
        }

        val attitude2 = attitudeSlot.captured
        assertFalse(rollSlot.isCaptured)
        assertFalse(pitchSlot.isCaptured)
        assertFalse(yawSlot.isCaptured)
        assertFalse(cSlot.isCaptured)
        assertFalse(quaternionCovarianceSlot.isCaptured)
        assertFalse(eulerAnglesCovarianceSlot.isCaptured)

        assertEquals(nedAttitude, attitude2)
    }

    @Test
    fun collectorMeasurementListener_whenUnreliableAndDurationNotExceeded_processesMeasurement() {

        val estimator = KalmanRelativeAttitudeEstimator(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            estimateCovariances = false,
            accuracyChangedListener = accuracyChangedListener
        )

        val processor: KalmanRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("processor")
        requireNotNull(processor)
        val processorSpy = spyk(processor)
        estimator.setPrivateProperty("processor", processorSpy)

        every { processorSpy.process(any()) }.returns(true)
        val randomizer = UniformRandomizer()
        val nedAttitude = Quaternion(
            randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES),
            randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES),
            randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        )
        every { processorSpy.nedAttitude }.returns(nedAttitude)

        estimator.setPrivateProperty("unreliable", true)

        val timestamp = System.nanoTime()
        estimator.setPrivateProperty("unreliableTimestamp", timestamp)

        // call listener
        val collector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)

        val measurementListener = collector.measurementListener
        requireNotNull(measurementListener)

        val measurement = AccelerometerAndGyroscopeSyncedSensorMeasurement(timestamp = timestamp)
        measurementListener.onMeasurement(collector, measurement)

        // check
        val unreliable: Boolean? = estimator.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertTrue(unreliable)

        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun collectorMeasurementListener_whenUnreliableAndDurationExceededWithListener_processesMeasurementAndNotifies() {

        justRun { accuracyChangedListener.onAccuracyChanged(any(), any(), any()) }

        val estimator = KalmanRelativeAttitudeEstimator(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            estimateCovariances = false,
            accuracyChangedListener = accuracyChangedListener
        )

        val processor: KalmanRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("processor")
        requireNotNull(processor)
        val processorSpy = spyk(processor)
        estimator.setPrivateProperty("processor", processorSpy)

        every { processorSpy.process(any()) }.returns(true)
        val randomizer = UniformRandomizer()
        val nedAttitude = Quaternion(
            randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES),
            randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES),
            randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        )
        every { processorSpy.nedAttitude }.returns(nedAttitude)

        estimator.setPrivateProperty("unreliable", true)

        val timestamp = System.nanoTime()
        val unreliableTimestamp = timestamp - 2_000_000_000L
        estimator.setPrivateProperty("unreliableTimestamp", unreliableTimestamp)

        estimator.setPrivateProperty("sensorAccuracy", SensorAccuracy.HIGH)

        // call listener
        val collector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)

        val measurementListener = collector.measurementListener
        requireNotNull(measurementListener)

        val measurement = AccelerometerAndGyroscopeSyncedSensorMeasurement(timestamp = timestamp)
        measurementListener.onMeasurement(collector, measurement)

        // check
        val unreliable: Boolean? = estimator.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                estimator,
                SensorType.RELATIVE_ATTITUDE,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun collectorMeasurementListener_whenUnreliableAndDurationExceededAndNoListener_processesMeasurement() {

        val estimator = KalmanRelativeAttitudeEstimator(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            estimateCovariances = false
        )

        val processor: KalmanRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("processor")
        requireNotNull(processor)
        val processorSpy = spyk(processor)
        estimator.setPrivateProperty("processor", processorSpy)

        every { processorSpy.process(any()) }.returns(true)
        val randomizer = UniformRandomizer()
        val nedAttitude = Quaternion(
            randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES),
            randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES),
            randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        )
        every { processorSpy.nedAttitude }.returns(nedAttitude)

        estimator.setPrivateProperty("unreliable", true)

        val timestamp = System.nanoTime()
        val unreliableTimestamp = timestamp - 2_000_000_000L
        estimator.setPrivateProperty("unreliableTimestamp", unreliableTimestamp)

        estimator.setPrivateProperty("sensorAccuracy", SensorAccuracy.HIGH)

        // call listener
        val collector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)

        val measurementListener = collector.measurementListener
        requireNotNull(measurementListener)

        val measurement = AccelerometerAndGyroscopeSyncedSensorMeasurement(timestamp = timestamp)
        measurementListener.onMeasurement(collector, measurement)

        // check
        val unreliable: Boolean? = estimator.getPrivateProperty("unreliable")
        requireNotNull(unreliable)
        assertFalse(unreliable)

        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun collectorAccuracyChangedListener_whenNoListener_setsAccuracy() {
        val estimator = KalmanRelativeAttitudeEstimator(context)

        val collector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)

        val accuracyChangedListener = collector.accuracyChangedListener
        requireNotNull(accuracyChangedListener)
        accuracyChangedListener.onAccuracyChanged(
            collector,
            SensorType.RELATIVE_ATTITUDE,
            SensorAccuracy.HIGH
        )

        val sensorAccuracy: SensorAccuracy? = estimator.getPrivateProperty("sensorAccuracy")
        requireNotNull(sensorAccuracy)
        assertEquals(SensorAccuracy.HIGH, sensorAccuracy)
    }

    @Test
    fun collectorAccuracyChangedListener_whenListener_setsAccuracyAndNotifies() {
        justRun { accuracyChangedListener.onAccuracyChanged(any(), any(), any()) }

        val estimator = KalmanRelativeAttitudeEstimator(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val collector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)

        val listener = collector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            collector,
            SensorType.RELATIVE_ATTITUDE,
            SensorAccuracy.HIGH
        )

        val sensorAccuracy: SensorAccuracy? = estimator.getPrivateProperty("sensorAccuracy")
        requireNotNull(sensorAccuracy)
        assertEquals(SensorAccuracy.HIGH, sensorAccuracy)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                estimator,
                SensorType.RELATIVE_ATTITUDE,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun start_whenRunning_throwsIllegalStateException() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            val estimator = KalmanRelativeAttitudeEstimator(context)

            // set as running
            estimator.setPrivateProperty("running", true)

            assertTrue(estimator.running)

            assertThrows(IllegalStateException::class.java) {
                estimator.start()
            }
        }
    }

    @Test
    fun start_whenNotRunning_resetsAndStartsCollector() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            val estimator = KalmanRelativeAttitudeEstimator(context)

            // setup
            val collector: AccelerometerAndGyroscopeSyncedSensorCollector? =
                estimator.getPrivateProperty("collector")
            requireNotNull(collector)
            val collectorSpy = spyk(collector)
            every { collectorSpy.start(any()) }.returns(true)
            estimator.setPrivateProperty("collector", collectorSpy)

            estimator.setPrivateProperty("lastTimestamp", 2L)
            estimator.setPrivateProperty("unreliableTimestamp", 3L)

            val processor: KalmanRelativeAttitudeProcessor? =
                estimator.getPrivateProperty("processor")
            requireNotNull(processor)
            val processorSpy = spyk(processor)
            justRun { processorSpy.reset() }
            estimator.setPrivateProperty("processor", processorSpy)

            assertFalse(estimator.running)

            // start
            estimator.start()

            assertTrue(estimator.running)
            verify(exactly = 1) { collectorSpy.start(startTimestamp) }
            verify(exactly = 1) { processorSpy.reset() }

            val lastTimestamp: Long? = estimator.getPrivateProperty("lastTimestamp")
            requireNotNull(lastTimestamp)
            assertEquals(-1L, lastTimestamp)

            val unreliableTimestamp: Long? = estimator.getPrivateProperty("unreliableTimestamp")
            requireNotNull(unreliableTimestamp)
            assertEquals(-1L, unreliableTimestamp)
        }
    }

    @Test
    fun stop_stopsCollectorAndResets() {
        val estimator = KalmanRelativeAttitudeEstimator(context)

        // setup
        val collector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("collector")
        requireNotNull(collector)
        val collectorSpy = spyk(collector)
        justRun { collectorSpy.stop() }
        estimator.setPrivateProperty("collector", collectorSpy)

        estimator.setPrivateProperty("lastTimestamp", 2L)
        estimator.setPrivateProperty("unreliableTimestamp", 3L)

        val processor: KalmanRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("processor")
        requireNotNull(processor)
        val processorSpy = spyk(processor)
        justRun { processorSpy.reset() }
        estimator.setPrivateProperty("processor", processorSpy)

        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        // stop
        estimator.stop()

        verify { collectorSpy.stop() }
        verify(exactly = 1) { processorSpy.reset() }

        assertFalse(estimator.running)

        val lastTimestamp: Long? = estimator.getPrivateProperty("lastTimestamp")
        requireNotNull(lastTimestamp)
        assertEquals(-1L, lastTimestamp)

        val unreliableTimestamp: Long? = estimator.getPrivateProperty("unreliableTimestamp")
        requireNotNull(unreliableTimestamp)
        assertEquals(-1L, unreliableTimestamp)
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