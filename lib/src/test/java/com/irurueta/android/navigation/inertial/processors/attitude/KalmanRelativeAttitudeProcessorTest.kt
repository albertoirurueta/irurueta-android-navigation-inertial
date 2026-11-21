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

package com.irurueta.android.navigation.inertial.processors.attitude

import android.location.Location
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerAndGyroscopeSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem
import com.irurueta.android.navigation.inertial.processors.attitude.KalmanRelativeAttitudeProcessor.OnNumericalInstabilitiesDetectedListener
import com.irurueta.android.navigation.inertial.processors.attitude.KalmanRelativeAttitudeProcessor.OnProcessedListener
import com.irurueta.android.navigation.inertial.processors.filters.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.processors.filters.MeanAveragingFilter
import com.irurueta.android.testutils.callPrivateFunc
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegrator
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegratorType
import com.irurueta.numerical.signal.processing.KalmanFilter
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import com.irurueta.units.TimeConverter
import io.mockk.every
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import io.mockk.justRun
import io.mockk.slot
import io.mockk.spyk
import io.mockk.verify
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test

class KalmanRelativeAttitudeProcessorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxed = true)
    private lateinit var location: Location

    @MockK
    private lateinit var processorListener: OnProcessedListener

    @MockK
    private lateinit var numericalInstabilitiesListener: OnNumericalInstabilitiesDetectedListener

    @Test
    fun constructor_whenDefaultValues_setsExpectedValues() {
        val processor = KalmanRelativeAttitudeProcessor()

        // check
        assertTrue(processor.averagingFilter is LowPassAveragingFilter)
        assertNull(processor.location)
        assertTrue(processor.adjustGravityNorm)
        assertTrue(processor.computeEulerAngles)
        assertTrue(processor.computeCovariances)
        assertTrue(processor.computeQuaternionCovariance)
        assertTrue(processor.computeEulerAnglesCovariance)
        assertTrue(processor.computeEulerAnglesCovariance)
        assertTrue(processor.computeAccelerometerCovariance)
        assertTrue(processor.computeRotationAxisCovariance)
        assertEquals(
            KalmanRelativeAttitudeProcessor.DEFAULT_GYROSCOPE_NOISE_PSD,
            processor.gyroscopeNoisePsd,
            0.0
        )
        assertEquals(
            KalmanRelativeAttitudeProcessor.DEFAULT_ACCELEROMETER_NOISE_STANDARD_DEVIATION,
            processor.accelerometerNoiseStandardDeviation,
            0.0
        )
        assertEquals(
            QuaternionStepIntegrator.DEFAULT_TYPE,
            processor.quaternionStepIntegratorType
        )
        assertEquals(
            KalmanRelativeAttitudeProcessor.DEFAULT_FREE_FALL_THRESHOLD,
            processor.freeFallThreshold,
            0.0
        )
        assertEquals(
            KalmanRelativeAttitudeProcessor.DEFAULT_SYMMETRIC_THRESHOLD,
            processor.symmetricThreshold,
            0.0
        )
        assertNull(processor.processorListener)
        assertNull(processor.numericalInstabilitiesListener)

        assertEquals(Quaternion(), processor.nedAttitude)
        assertEquals(AccelerationTriad(), processor.acceleration)
        assertArrayEquals(
            DoubleArray(AccelerationTriad.COMPONENTS),
            processor.rotationAxis,
            0.0
        )
        assertEquals(0.0, processor.measurementsTimeIntervalSeconds, 0.0)
        assertArrayEquals(
            DoubleArray(Quaternion.N_ANGLES),
            processor.eulerAngles,
            0.0
        )
        assertEquals(
            Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS),
            processor.nedAttitudeCovariance
        )
        assertEquals(
            Matrix(Quaternion.N_ANGLES, Quaternion.N_ANGLES),
            processor.eulerAnglesCovariance
        )
        assertEquals(
            Matrix(AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS),
            processor.accelerometerCovariance
        )
        assertEquals(
            Matrix(AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS),
            processor.rotationAxisCovariance
        )
    }

    @Test
    fun constructor_whenNonDefaultValues_setsExpectedValues() {
        val averagingFilter =
            MeanAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()
        val location = getLocation()
        val randomizer = UniformRandomizer()
        val gyroscopeNoisePsd = randomizer.nextDouble()
        val accelerometerNoiseStandardDeviation = randomizer.nextDouble()
        val freeFallThreshold = randomizer.nextDouble()
        val symmetricThreshold = randomizer.nextDouble()

        val processor = KalmanRelativeAttitudeProcessor(
            averagingFilter,
            location,
            adjustGravityNorm = false,
            computeEulerAngles = false,
            computeCovariances = false,
            computeQuaternionCovariance = false,
            computeEulerAnglesCovariance = false,
            computeAccelerometerCovariance = false,
            computeRotationAxisCovariance = false,
            gyroscopeNoisePsd,
            accelerometerNoiseStandardDeviation,
            QuaternionStepIntegratorType.EULER_METHOD,
            freeFallThreshold,
            symmetricThreshold,
            processorListener,
            numericalInstabilitiesListener
        )

        // check
        assertSame(averagingFilter, processor.averagingFilter)
        assertSame(location, processor.location)
        assertFalse(processor.adjustGravityNorm)
        assertFalse(processor.computeEulerAngles)
        assertFalse(processor.computeCovariances)
        assertFalse(processor.computeQuaternionCovariance)
        assertFalse(processor.computeEulerAnglesCovariance)
        assertFalse(processor.computeAccelerometerCovariance)
        assertFalse(processor.computeRotationAxisCovariance)
        assertEquals(
            gyroscopeNoisePsd,
            processor.gyroscopeNoisePsd,
            0.0
        )
        assertEquals(
            accelerometerNoiseStandardDeviation,
            processor.accelerometerNoiseStandardDeviation,
            0.0
        )
        assertEquals(
            QuaternionStepIntegratorType.EULER_METHOD,
            processor.quaternionStepIntegratorType
        )
        assertEquals(
            freeFallThreshold,
            processor.freeFallThreshold,
            0.0
        )
        assertEquals(
            symmetricThreshold,
            processor.symmetricThreshold,
            0.0
        )
        assertSame(processorListener, processor.processorListener)
        assertSame(
            numericalInstabilitiesListener,
            processor.numericalInstabilitiesListener
        )

        assertEquals(Quaternion(), processor.nedAttitude)
        assertEquals(AccelerationTriad(), processor.acceleration)
        assertArrayEquals(
            DoubleArray(AccelerationTriad.COMPONENTS),
            processor.rotationAxis,
            0.0
        )
        assertEquals(0.0, processor.measurementsTimeIntervalSeconds, 0.0)
        assertNull(processor.eulerAngles)
        assertNull(processor.nedAttitudeCovariance)
        assertNull(processor.eulerAnglesCovariance)
        assertNull(processor.accelerometerCovariance)
        assertNull(processor.rotationAxisCovariance)
    }

    @Test
    fun eulerAngles_whenComputeCovariancesAndComputeEulerAngles_returnsExpectedResult() {
        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = true,
            computeEulerAngles = true
        )

        assertArrayEquals(
            DoubleArray(Quaternion.N_ANGLES),
            processor.eulerAngles,
            0.0
        )
    }

    @Test
    fun eulerAngles_whenNoComputeCovariances_returnsNull() {
        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = false,
            computeEulerAngles = true
        )

        assertNull(processor.eulerAngles)
    }

    @Test
    fun eulerAngles_whenNoComputeEulerAngles_returnsNull() {
        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = true,
            computeEulerAngles = false
        )

        assertNull(processor.eulerAngles)
    }

    @Test
    fun eulerAngles_whenNoComputeCovariancesAndNoComputeEulerAngles_returnsNull() {
        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = false,
            computeEulerAngles = false
        )

        assertNull(processor.eulerAngles)
    }

    @Test
    fun nedAttitudeCovariance_whenComputeCovariancesAndComputeQuaternionCovariance_returnsExpectedResult() {
        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = true,
            computeQuaternionCovariance = true
        )

        assertEquals(
            Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS),
            processor.nedAttitudeCovariance
        )
    }

    @Test
    fun nedAttitudeCovariance_whenNoComputeCovariances_returnsNull() {
        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = false,
            computeQuaternionCovariance = true
        )

        assertNull(processor.nedAttitudeCovariance)
    }

    @Test
    fun nedAttitudeCovariance_whenNoComputeQuaternionCovariance_returnsNull() {
        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = true,
            computeQuaternionCovariance = false
        )

        assertNull(processor.nedAttitudeCovariance)
    }

    @Test
    fun nedAttitudeCovariance_whenNoComputeCovariancesAndNoComputeQuaternionCovariance_returnsNull() {
        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = false,
            computeQuaternionCovariance = false
        )

        assertNull(processor.nedAttitudeCovariance)
    }

    @Test
    fun eulerAnglesCovariance_whenComputeCovariancesAndComputeEulerAnglesCovariance_ReturnsExpectedResult() {
        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = true,
            computeEulerAnglesCovariance = true
        )

        assertEquals(
            Matrix(Quaternion.N_ANGLES, Quaternion.N_ANGLES),
            processor.eulerAnglesCovariance
        )
    }

    @Test
    fun eulerAnglesCovariance_whenNoComputeCovariances_returnsNull() {
        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = false,
            computeEulerAnglesCovariance = true
        )

        assertNull(processor.eulerAnglesCovariance)
    }

    @Test
    fun eulerAnglesCovariance_whenNoComputeEulerAnglesCovariance_returnsNull() {
        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = true,
            computeEulerAnglesCovariance = false
        )

        assertNull(processor.eulerAnglesCovariance)
    }

    @Test
    fun eulerAnglesCovariance_whenNoComputeCovariancesAndNoComputeEulerAnglesCovariance_returnsNull() {
        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = false,
            computeEulerAnglesCovariance = false
        )

        assertNull(processor.eulerAnglesCovariance)
    }

    @Test
    fun accelerometerCovariance_whenComputeCovariancesAndComputeAccelerometerCovariance_returnsExpectedResult() {
        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = true,
            computeAccelerometerCovariance = true
        )

        assertEquals(
            Matrix(AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS),
            processor.accelerometerCovariance
        )
    }

    @Test
    fun accelerometerCovariance_whenNoComputeCovariances_returnsNull() {
        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = false,
            computeAccelerometerCovariance = true
        )

        assertNull(processor.accelerometerCovariance)
    }

    @Test
    fun accelerometerCovariance_whenNoComputeAccelerometerCovariance_returnsNull() {
        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = true,
            computeAccelerometerCovariance = false
        )

        assertNull(processor.accelerometerCovariance)
    }

    @Test
    fun accelerometerCovariance_whenNoComputeCovariancesAndNoComputeAccelerometerCovariance_returnsNull() {
        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = false,
            computeAccelerometerCovariance = false
        )

        assertNull(processor.accelerometerCovariance)
    }

    @Test
    fun rotationAxisCovariance_whenComputeCovariancesAndComputeRotationAxisCovariance_returnsExpectedResult() {
        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = true,
            computeRotationAxisCovariance = true
        )

        assertEquals(
            Matrix(AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS),
            processor.rotationAxisCovariance
        )
    }

    @Test
    fun rotationAxisCovariance_whenNoComputeCovariances_returnsNull() {
        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = false,
            computeRotationAxisCovariance = true
        )

        assertNull(processor.rotationAxisCovariance)
    }

    @Test
    fun rotationAxisCovariance_whenNoComputeRotationAxisCovariance_returnsNull() {
        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = true,
            computeRotationAxisCovariance = false
        )

        assertNull(processor.rotationAxisCovariance)
    }

    @Test
    fun rotationAxisCovariance_whenNoComputeCovariancesAndNoComputeRotationAxisCovariance_ReturnsNull() {
        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = false,
            computeRotationAxisCovariance = false
        )

        assertNull(processor.rotationAxisCovariance)
    }

    @Test
    fun location_setsGravityProcessorLocation() {
        val processor = KalmanRelativeAttitudeProcessor()

        // check default value
        assertNull(processor.location)

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        assertNull(gravityProcessor.location)

        // set new value
        val location = getLocation()
        processor.location = location

        // check
        assertSame(location, processor.location)
        assertSame(location, gravityProcessor.location)
    }

    @Test
    fun adjustGravityNorm_setsGravityProcessorAdjustGravityNorm() {
        val processor = KalmanRelativeAttitudeProcessor()

        assertTrue(processor.adjustGravityNorm)

        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        assertTrue(gravityProcessor.adjustGravityNorm)

        // set new value
        processor.adjustGravityNorm = false

        // check
        assertFalse(processor.adjustGravityNorm)
        assertFalse(gravityProcessor.adjustGravityNorm)
    }

    @Test
    fun process_whenEmptyMeasurement_returnsFalse() {
        val processor = KalmanRelativeAttitudeProcessor()

        val measurement = AccelerometerAndGyroscopeSyncedSensorMeasurement()
        assertFalse(processor.process(measurement))
    }

    @Test
    fun process_whenMissingGyroscopeMeasurement_returnsFalse() {
        val processor = KalmanRelativeAttitudeProcessor()

        val measurement = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            AccelerometerSensorMeasurement()
        )
        assertFalse(processor.process(measurement))
    }

    @Test
    fun process_whenFirstMeasurement_returnsFalse() {
        val processor = KalmanRelativeAttitudeProcessor()

        val previousTimestamp1: Long? = processor.getPrivateProperty("previousTimestamp")
        assertEquals(-1L, previousTimestamp1)

        val timestamp = System.nanoTime()
        val accelerometerMeasurement = createAccelerometerMeasurement(timestamp)
        val gyroscopeMeasurement = createGyroscopeMeasurement(timestamp)
        val measurement = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gyroscopeMeasurement,
            timestamp
        )
        assertFalse(processor.process(measurement))

        // check expected updates
        val previousTimestamp2: Long? = processor.getPrivateProperty("previousTimestamp")
        assertEquals(timestamp, previousTimestamp2)
    }

    @Test
    fun process_whenSecondMeasurementAndGravityProcessorNotInitialized_returnsFalse() {
        val processor = KalmanRelativeAttitudeProcessor()

        val timestamp1 = System.nanoTime()
        val accelerometerMeasurement1 = createAccelerometerMeasurement(timestamp1)
        val gyroscopeMeasurement1 = createGyroscopeMeasurement(timestamp1)
        val measurement1 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement1,
            gyroscopeMeasurement1,
            timestamp1
        )
        assertFalse(processor.process(measurement1))

        val previousTimestamp1: Long? = processor.getPrivateProperty("previousTimestamp")
        requireNotNull(previousTimestamp1)
        assertEquals(timestamp1, previousTimestamp1)

        val timestamp2 = System.nanoTime()
        val accelerometerMeasurement2 = createAccelerometerMeasurement(timestamp2)
        val gyroscopeMeasurement2 = createGyroscopeMeasurement(timestamp2)
        val measurement2 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement2,
            gyroscopeMeasurement2,
            timestamp2
        )
        assertFalse(processor.process(measurement2))

        val previousTimestamp2: Long? = processor.getPrivateProperty("previousTimestamp")
        requireNotNull(previousTimestamp2)
        assertEquals(timestamp2, previousTimestamp2)

        assertEquals(
            TimeConverter.nanosecondToSecond((previousTimestamp2 - previousTimestamp1).toDouble()),
            processor.measurementsTimeIntervalSeconds,
            0.0
        )

        val nedGyroscopeMeasurement: GyroscopeSensorMeasurement? =
            processor.getPrivateProperty("nedGyroscopeMeasurement")
        requireNotNull(nedGyroscopeMeasurement)
        assertEquals(
            gyroscopeMeasurement2.toNed(),
            nedGyroscopeMeasurement
        )

        val nedAngularSpeedTriad: AngularSpeedTriad? =
            processor.getPrivateProperty("nedAngularSpeedTriad")
        requireNotNull(nedAngularSpeedTriad)
        assertEquals(
            gyroscopeMeasurement2.toNed().toTriad(),
            nedAngularSpeedTriad
        )

        val nedAccelerometerMeasurement: AccelerometerSensorMeasurement? =
            processor.getPrivateProperty("nedAccelerometerMeasurement")
        requireNotNull(nedAccelerometerMeasurement)
        assertEquals(
            accelerometerMeasurement2.toNed(),
            nedAccelerometerMeasurement
        )

        val nedAccelerationTriad: AccelerationTriad? =
            processor.getPrivateProperty("nedAccelerationTriad")
        requireNotNull(nedAccelerationTriad)
        assertEquals(
            accelerometerMeasurement2.toNed().toTriad(),
            nedAccelerationTriad
        )

        val previousNedAngularSpeedTriad: AngularSpeedTriad? =
            processor.getPrivateProperty("previousNedAngularSpeedTriad")
        requireNotNull(previousNedAngularSpeedTriad)
        assertEquals(previousNedAngularSpeedTriad, nedAngularSpeedTriad)
    }

    @Test
    fun process_whenThirdMeasurement_returnsTrue() {
        val processor = KalmanRelativeAttitudeProcessor()

        val timestamp1 = System.nanoTime()
        val accelerometerMeasurement1 = createAccelerometerMeasurement(timestamp1)
        val gyroscopeMeasurement1 = createGyroscopeMeasurement(timestamp1)
        val measurement1 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement1,
            gyroscopeMeasurement1,
            timestamp1
        )
        assertFalse(processor.process(measurement1))

        val initialized1: Boolean? = processor.getPrivateProperty("initialized")
        requireNotNull(initialized1)
        assertFalse(initialized1)

        val timestamp2 = System.nanoTime()
        val accelerometerMeasurement2 = createAccelerometerMeasurement(timestamp2)
        val gyroscopeMeasurement2 = createGyroscopeMeasurement(timestamp2)
        val measurement2 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement2,
            gyroscopeMeasurement2,
            timestamp2
        )
        assertFalse(processor.process(measurement2))

        val initialized2: Boolean? = processor.getPrivateProperty("initialized")
        requireNotNull(initialized2)
        assertFalse(initialized2)

        val timestamp3 = System.nanoTime()
        val accelerometerMeasurement3 = createAccelerometerMeasurement(timestamp3)
        val gyroscopeMeasurement3 = createGyroscopeMeasurement(timestamp3)
        val measurement3 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement3,
            gyroscopeMeasurement3,
            timestamp3
        )
        assertTrue(processor.process(measurement3))

        val initialized3: Boolean? = processor.getPrivateProperty("initialized")
        requireNotNull(initialized3)
        assertTrue(initialized3)
    }

    @Test
    fun process_whenFourthMeasurement_returnsTrue() {
        val processor = KalmanRelativeAttitudeProcessor()

        val timestamp1 = System.nanoTime()
        val accelerometerMeasurement1 = createAccelerometerMeasurement(timestamp1)
        val gyroscopeMeasurement1 = createGyroscopeMeasurement(timestamp1)
        val measurement1 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement1,
            gyroscopeMeasurement1,
            timestamp1
        )
        assertFalse(processor.process(measurement1))

        val timestamp2 = System.nanoTime()
        val accelerometerMeasurement2 = createAccelerometerMeasurement(timestamp2)
        val gyroscopeMeasurement2 = createGyroscopeMeasurement(timestamp2)
        val measurement2 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement2,
            gyroscopeMeasurement2,
            timestamp2
        )
        assertFalse(processor.process(measurement2))

        val timestamp3 = System.nanoTime()
        val accelerometerMeasurement3 = createAccelerometerMeasurement(timestamp3)
        val gyroscopeMeasurement3 = createGyroscopeMeasurement(timestamp3)
        val measurement3 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement3,
            gyroscopeMeasurement3,
            timestamp3
        )
        assertTrue(processor.process(measurement3))

        val timestamp4 = System.nanoTime()
        val accelerometerMeasurement4 = createAccelerometerMeasurement(timestamp4)
        val gyroscopeMeasurement4 = createGyroscopeMeasurement(timestamp4)
        val measurement4 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement4,
            gyroscopeMeasurement4,
            timestamp4
        )
        assertTrue(processor.process(measurement4))
    }

    @Test
    fun process_whenFreeFall_returnsTrue() {
        val processor = KalmanRelativeAttitudeProcessor(adjustGravityNorm = false)

        val timestamp1 = System.nanoTime()
        val accelerometerMeasurement1 = createZeroAccelerometerMeasurement(timestamp1)
        val gyroscopeMeasurement1 = createGyroscopeMeasurement(timestamp1)
        val measurement1 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement1,
            gyroscopeMeasurement1,
            timestamp1
        )
        assertFalse(processor.process(measurement1))

        val timestamp2 = System.nanoTime()
        val accelerometerMeasurement2 = createZeroAccelerometerMeasurement(timestamp2)
        val gyroscopeMeasurement2 = createGyroscopeMeasurement(timestamp2)
        val measurement2 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement2,
            gyroscopeMeasurement2,
            timestamp2
        )
        assertFalse(processor.process(measurement2))

        val timestamp3 = System.nanoTime()
        val accelerometerMeasurement3 = createZeroAccelerometerMeasurement(timestamp3)
        val gyroscopeMeasurement3 = createGyroscopeMeasurement(timestamp3)
        val measurement3 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement3,
            gyroscopeMeasurement3,
            timestamp3
        )
        assertTrue(processor.process(measurement3))

        val timestamp4 = System.nanoTime()
        val accelerometerMeasurement4 = createZeroAccelerometerMeasurement(timestamp4)
        val gyroscopeMeasurement4 = createGyroscopeMeasurement(timestamp4)
        val measurement4 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement4,
            gyroscopeMeasurement4,
            timestamp4
        )
        assertTrue(processor.process(measurement4))
    }

    @Test
    fun process_whenListener_notifies() {
        justRun {
            processorListener.onProcessed(
                any(),
                any(),
                any(),
                any(),
                any(),
                any(),
                any()
            )
        }

        val processor = KalmanRelativeAttitudeProcessor(
            processorListener = processorListener
        )

        val timestamp1 = System.nanoTime()
        val accelerometerMeasurement1 = createAccelerometerMeasurement(timestamp1)
        val gyroscopeMeasurement1 = createGyroscopeMeasurement(timestamp1)
        val measurement1 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement1,
            gyroscopeMeasurement1,
            timestamp1
        )
        assertFalse(processor.process(measurement1))

        val timestamp2 = System.nanoTime()
        val accelerometerMeasurement2 = createAccelerometerMeasurement(timestamp2)
        val gyroscopeMeasurement2 = createGyroscopeMeasurement(timestamp2)
        val measurement2 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement2,
            gyroscopeMeasurement2,
            timestamp2
        )
        assertFalse(processor.process(measurement2))

        val timestamp3 = System.nanoTime()
        val accelerometerMeasurement3 = createAccelerometerMeasurement(timestamp3)
        val gyroscopeMeasurement3 = createGyroscopeMeasurement(timestamp3)
        val measurement3 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement3,
            gyroscopeMeasurement3,
            timestamp3
        )
        assertTrue(processor.process(measurement3))

        val timestamp4 = System.nanoTime()
        val accelerometerMeasurement4 = createAccelerometerMeasurement(timestamp4)
        val gyroscopeMeasurement4 = createGyroscopeMeasurement(timestamp4)
        val measurement4 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement4,
            gyroscopeMeasurement4,
            timestamp4
        )
        assertTrue(processor.process(measurement4))

        val nedAttitudeSlot = slot<Quaternion>()
        val eulerAnglesSlot = slot<DoubleArray>()
        val nedAttitudeCovarianceSlot = slot<Matrix>()
        val eulerAnglesCovarianceSlot = slot<Matrix>()
        val accelerometerCovarianceSlot = slot<Matrix>()
        val rotationAxisCovarianceSlot = slot<Matrix>()

        verify(exactly = 1) {
            processorListener.onProcessed(
                processor,
                capture(nedAttitudeSlot),
                capture(eulerAnglesSlot),
                capture(nedAttitudeCovarianceSlot),
                capture(eulerAnglesCovarianceSlot),
                capture(accelerometerCovarianceSlot),
                capture(rotationAxisCovarianceSlot)
            )
        }

        val nedAttitude = nedAttitudeSlot.captured
        val eulerAngles = eulerAnglesSlot.captured
        val nedAttitudeCovariance = nedAttitudeCovarianceSlot.captured
        val eulerAnglesCovariance = eulerAnglesCovarianceSlot.captured
        val accelerometerCovariance = accelerometerCovarianceSlot.captured
        val rotationAxisCovariance = rotationAxisCovarianceSlot.captured

        assertEquals(nedAttitude, processor.nedAttitude)
        assertEquals(eulerAngles, processor.eulerAngles)
        assertEquals(nedAttitudeCovariance, processor.nedAttitudeCovariance)
        assertEquals(eulerAnglesCovariance, processor.eulerAnglesCovariance)
        assertEquals(accelerometerCovariance, processor.accelerometerCovariance)
        assertEquals(rotationAxisCovariance, processor.rotationAxisCovariance)
    }

    @Test
    fun process_whenListenerAndComputeCovariancesOnly_notifies() {
        justRun {
            processorListener.onProcessed(
                any(),
                any(),
                any(),
                any(),
                any(),
                any(),
                any()
            )
        }

        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = true,
            computeEulerAngles = false,
            computeEulerAnglesCovariance = false,
            processorListener = processorListener
        )

        val timestamp1 = System.nanoTime()
        val accelerometerMeasurement1 = createAccelerometerMeasurement(timestamp1)
        val gyroscopeMeasurement1 = createGyroscopeMeasurement(timestamp1)
        val measurement1 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement1,
            gyroscopeMeasurement1,
            timestamp1
        )
        assertFalse(processor.process(measurement1))

        val timestamp2 = System.nanoTime()
        val accelerometerMeasurement2 = createAccelerometerMeasurement(timestamp2)
        val gyroscopeMeasurement2 = createGyroscopeMeasurement(timestamp2)
        val measurement2 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement2,
            gyroscopeMeasurement2,
            timestamp2
        )
        assertFalse(processor.process(measurement2))

        val timestamp3 = System.nanoTime()
        val accelerometerMeasurement3 = createAccelerometerMeasurement(timestamp3)
        val gyroscopeMeasurement3 = createGyroscopeMeasurement(timestamp3)
        val measurement3 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement3,
            gyroscopeMeasurement3,
            timestamp3
        )
        assertTrue(processor.process(measurement3))

        val timestamp4 = System.nanoTime()
        val accelerometerMeasurement4 = createAccelerometerMeasurement(timestamp4)
        val gyroscopeMeasurement4 = createGyroscopeMeasurement(timestamp4)
        val measurement4 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement4,
            gyroscopeMeasurement4,
            timestamp4
        )
        assertTrue(processor.process(measurement4))

        val nedAttitudeSlot = slot<Quaternion>()
        val eulerAnglesSlot = slot<DoubleArray>()
        val nedAttitudeCovarianceSlot = slot<Matrix>()
        val eulerAnglesCovarianceSlot = slot<Matrix>()
        val accelerometerCovarianceSlot = slot<Matrix>()
        val rotationAxisCovarianceSlot = slot<Matrix>()

        verify(exactly = 1) {
            processorListener.onProcessed(
                processor,
                capture(nedAttitudeSlot),
                capture(eulerAnglesSlot),
                capture(nedAttitudeCovarianceSlot),
                capture(eulerAnglesCovarianceSlot),
                capture(accelerometerCovarianceSlot),
                capture(rotationAxisCovarianceSlot)
            )
        }

        val nedAttitude = nedAttitudeSlot.captured
        val nedAttitudeCovariance = nedAttitudeCovarianceSlot.captured
        val accelerometerCovariance = accelerometerCovarianceSlot.captured
        val rotationAxisCovariance = rotationAxisCovarianceSlot.captured

        assertEquals(nedAttitude, processor.nedAttitude)
        assertNull(processor.eulerAngles)
        assertEquals(nedAttitudeCovariance, processor.nedAttitudeCovariance)
        assertNull(processor.eulerAnglesCovariance)
        assertEquals(accelerometerCovariance, processor.accelerometerCovariance)
        assertEquals(rotationAxisCovariance, processor.rotationAxisCovariance)
    }

    @Test
    fun process_whenListenerAndComputeEulerAnglesOnly_notifies() {
        justRun {
            processorListener.onProcessed(
                any(),
                any(),
                any(),
                any(),
                any(),
                any(),
                any()
            )
        }

        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = false,
            computeEulerAngles = true,
            computeEulerAnglesCovariance = false,
            processorListener = processorListener
        )

        val timestamp1 = System.nanoTime()
        val accelerometerMeasurement1 = createAccelerometerMeasurement(timestamp1)
        val gyroscopeMeasurement1 = createGyroscopeMeasurement(timestamp1)
        val measurement1 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement1,
            gyroscopeMeasurement1,
            timestamp1
        )
        assertFalse(processor.process(measurement1))

        val timestamp2 = System.nanoTime()
        val accelerometerMeasurement2 = createAccelerometerMeasurement(timestamp2)
        val gyroscopeMeasurement2 = createGyroscopeMeasurement(timestamp2)
        val measurement2 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement2,
            gyroscopeMeasurement2,
            timestamp2
        )
        assertFalse(processor.process(measurement2))

        val timestamp3 = System.nanoTime()
        val accelerometerMeasurement3 = createAccelerometerMeasurement(timestamp3)
        val gyroscopeMeasurement3 = createGyroscopeMeasurement(timestamp3)
        val measurement3 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement3,
            gyroscopeMeasurement3,
            timestamp3
        )
        assertTrue(processor.process(measurement3))

        val timestamp4 = System.nanoTime()
        val accelerometerMeasurement4 = createAccelerometerMeasurement(timestamp4)
        val gyroscopeMeasurement4 = createGyroscopeMeasurement(timestamp4)
        val measurement4 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement4,
            gyroscopeMeasurement4,
            timestamp4
        )
        assertTrue(processor.process(measurement4))

        val nedAttitudeSlot = slot<Quaternion>()
        val eulerAnglesSlot = slot<DoubleArray>()
        val nedAttitudeCovarianceSlot = slot<Matrix>()
        val eulerAnglesCovarianceSlot = slot<Matrix>()
        val accelerometerCovarianceSlot = slot<Matrix>()
        val rotationAxisCovarianceSlot = slot<Matrix>()

        verify(exactly = 1) {
            processorListener.onProcessed(
                processor,
                capture(nedAttitudeSlot),
                capture(eulerAnglesSlot),
                capture(nedAttitudeCovarianceSlot),
                capture(eulerAnglesCovarianceSlot),
                capture(accelerometerCovarianceSlot),
                capture(rotationAxisCovarianceSlot)
            )
        }

        val nedAttitude = nedAttitudeSlot.captured

        assertEquals(nedAttitude, processor.nedAttitude)
        assertNull(processor.eulerAngles)
        assertNull(processor.nedAttitudeCovariance)
        assertNull(processor.eulerAnglesCovariance)
        assertNull(processor.accelerometerCovariance)
        assertNull(processor.rotationAxisCovariance)
    }

    @Test
    fun process_whenListenerAndComputeEulerAnglesCovariance_notifies() {
        justRun {
            processorListener.onProcessed(
                any(),
                any(),
                any(),
                any(),
                any(),
                any(),
                any()
            )
        }

        val processor = KalmanRelativeAttitudeProcessor(
            computeCovariances = false,
            computeEulerAngles = false,
            computeEulerAnglesCovariance = true,
            processorListener = processorListener
        )

        val timestamp1 = System.nanoTime()
        val accelerometerMeasurement1 = createAccelerometerMeasurement(timestamp1)
        val gyroscopeMeasurement1 = createGyroscopeMeasurement(timestamp1)
        val measurement1 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement1,
            gyroscopeMeasurement1,
            timestamp1
        )
        assertFalse(processor.process(measurement1))

        val timestamp2 = System.nanoTime()
        val accelerometerMeasurement2 = createAccelerometerMeasurement(timestamp2)
        val gyroscopeMeasurement2 = createGyroscopeMeasurement(timestamp2)
        val measurement2 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement2,
            gyroscopeMeasurement2,
            timestamp2
        )
        assertFalse(processor.process(measurement2))

        val timestamp3 = System.nanoTime()
        val accelerometerMeasurement3 = createAccelerometerMeasurement(timestamp3)
        val gyroscopeMeasurement3 = createGyroscopeMeasurement(timestamp3)
        val measurement3 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement3,
            gyroscopeMeasurement3,
            timestamp3
        )
        assertTrue(processor.process(measurement3))

        val timestamp4 = System.nanoTime()
        val accelerometerMeasurement4 = createAccelerometerMeasurement(timestamp4)
        val gyroscopeMeasurement4 = createGyroscopeMeasurement(timestamp4)
        val measurement4 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement4,
            gyroscopeMeasurement4,
            timestamp4
        )
        assertTrue(processor.process(measurement4))

        val nedAttitudeSlot = slot<Quaternion>()
        val eulerAnglesSlot = slot<DoubleArray>()
        val nedAttitudeCovarianceSlot = slot<Matrix>()
        val eulerAnglesCovarianceSlot = slot<Matrix>()
        val accelerometerCovarianceSlot = slot<Matrix>()
        val rotationAxisCovarianceSlot = slot<Matrix>()

        verify(exactly = 1) {
            processorListener.onProcessed(
                processor,
                capture(nedAttitudeSlot),
                capture(eulerAnglesSlot),
                capture(nedAttitudeCovarianceSlot),
                capture(eulerAnglesCovarianceSlot),
                capture(accelerometerCovarianceSlot),
                capture(rotationAxisCovarianceSlot)
            )
        }

        val nedAttitude = nedAttitudeSlot.captured

        assertEquals(nedAttitude, processor.nedAttitude)
        assertNull(processor.eulerAngles)
        assertNull(processor.nedAttitudeCovariance)
        assertNull(processor.eulerAnglesCovariance)
        assertNull(processor.accelerometerCovariance)
        assertNull(processor.rotationAxisCovariance)
    }

    @Test
    fun process_whenListenerAndComputeAccelerometerCovarianceDisabled_notifies() {
        justRun {
            processorListener.onProcessed(
                any(),
                any(),
                any(),
                any(),
                any(),
                any(),
                any()
            )
        }

        val processor = KalmanRelativeAttitudeProcessor(
            computeAccelerometerCovariance = false,
            processorListener = processorListener
        )

        val timestamp1 = System.nanoTime()
        val accelerometerMeasurement1 = createAccelerometerMeasurement(timestamp1)
        val gyroscopeMeasurement1 = createGyroscopeMeasurement(timestamp1)
        val measurement1 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement1,
            gyroscopeMeasurement1,
            timestamp1
        )
        assertFalse(processor.process(measurement1))

        val timestamp2 = System.nanoTime()
        val accelerometerMeasurement2 = createAccelerometerMeasurement(timestamp2)
        val gyroscopeMeasurement2 = createGyroscopeMeasurement(timestamp2)
        val measurement2 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement2,
            gyroscopeMeasurement2,
            timestamp2
        )
        assertFalse(processor.process(measurement2))

        val timestamp3 = System.nanoTime()
        val accelerometerMeasurement3 = createAccelerometerMeasurement(timestamp3)
        val gyroscopeMeasurement3 = createGyroscopeMeasurement(timestamp3)
        val measurement3 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement3,
            gyroscopeMeasurement3,
            timestamp3
        )
        assertTrue(processor.process(measurement3))

        val timestamp4 = System.nanoTime()
        val accelerometerMeasurement4 = createAccelerometerMeasurement(timestamp4)
        val gyroscopeMeasurement4 = createGyroscopeMeasurement(timestamp4)
        val measurement4 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement4,
            gyroscopeMeasurement4,
            timestamp4
        )
        assertTrue(processor.process(measurement4))

        val nedAttitudeSlot = slot<Quaternion>()
        val eulerAnglesSlot = slot<DoubleArray>()
        val nedAttitudeCovarianceSlot = slot<Matrix>()
        val eulerAnglesCovarianceSlot = slot<Matrix>()
        val accelerometerCovarianceSlot = slot<Matrix>()
        val rotationAxisCovarianceSlot = slot<Matrix>()

        verify(exactly = 1) {
            processorListener.onProcessed(
                processor,
                capture(nedAttitudeSlot),
                capture(eulerAnglesSlot),
                capture(nedAttitudeCovarianceSlot),
                capture(eulerAnglesCovarianceSlot),
                capture(accelerometerCovarianceSlot),
                capture(rotationAxisCovarianceSlot)
            )
        }

        val nedAttitude = nedAttitudeSlot.captured
        val eulerAngles = eulerAnglesSlot.captured
        val nedAttitudeCovariance = nedAttitudeCovarianceSlot.captured
        val eulerAnglesCovariance = eulerAnglesCovarianceSlot.captured
        val rotationAxisCovariance = rotationAxisCovarianceSlot.captured

        assertEquals(nedAttitude, processor.nedAttitude)
        assertEquals(eulerAngles, processor.eulerAngles)
        assertEquals(nedAttitudeCovariance, processor.nedAttitudeCovariance)
        assertEquals(eulerAnglesCovariance, processor.eulerAnglesCovariance)
        assertNull(processor.accelerometerCovariance)
        assertEquals(rotationAxisCovariance, processor.rotationAxisCovariance)
    }

    @Test
    fun process_whenListenerAndComputeRotationAxisCovarianceDisabled_notifies() {
        justRun {
            processorListener.onProcessed(
                any(),
                any(),
                any(),
                any(),
                any(),
                any(),
                any()
            )
        }

        val processor = KalmanRelativeAttitudeProcessor(
            computeRotationAxisCovariance = false,
            processorListener = processorListener
        )

        val timestamp1 = System.nanoTime()
        val accelerometerMeasurement1 = createAccelerometerMeasurement(timestamp1)
        val gyroscopeMeasurement1 = createGyroscopeMeasurement(timestamp1)
        val measurement1 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement1,
            gyroscopeMeasurement1,
            timestamp1
        )
        assertFalse(processor.process(measurement1))

        val timestamp2 = System.nanoTime()
        val accelerometerMeasurement2 = createAccelerometerMeasurement(timestamp2)
        val gyroscopeMeasurement2 = createGyroscopeMeasurement(timestamp2)
        val measurement2 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement2,
            gyroscopeMeasurement2,
            timestamp2
        )
        assertFalse(processor.process(measurement2))

        val timestamp3 = System.nanoTime()
        val accelerometerMeasurement3 = createAccelerometerMeasurement(timestamp3)
        val gyroscopeMeasurement3 = createGyroscopeMeasurement(timestamp3)
        val measurement3 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement3,
            gyroscopeMeasurement3,
            timestamp3
        )
        assertTrue(processor.process(measurement3))

        val timestamp4 = System.nanoTime()
        val accelerometerMeasurement4 = createAccelerometerMeasurement(timestamp4)
        val gyroscopeMeasurement4 = createGyroscopeMeasurement(timestamp4)
        val measurement4 = AccelerometerAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement4,
            gyroscopeMeasurement4,
            timestamp4
        )
        assertTrue(processor.process(measurement4))

        val nedAttitudeSlot = slot<Quaternion>()
        val eulerAnglesSlot = slot<DoubleArray>()
        val nedAttitudeCovarianceSlot = slot<Matrix>()
        val eulerAnglesCovarianceSlot = slot<Matrix>()
        val accelerometerCovarianceSlot = slot<Matrix>()
        val rotationAxisCovarianceSlot = slot<Matrix>()

        verify(exactly = 1) {
            processorListener.onProcessed(
                processor,
                capture(nedAttitudeSlot),
                capture(eulerAnglesSlot),
                capture(nedAttitudeCovarianceSlot),
                capture(eulerAnglesCovarianceSlot),
                capture(accelerometerCovarianceSlot),
                capture(rotationAxisCovarianceSlot)
            )
        }

        val nedAttitude = nedAttitudeSlot.captured
        val eulerAngles = eulerAnglesSlot.captured
        val nedAttitudeCovariance = nedAttitudeCovarianceSlot.captured
        val eulerAnglesCovariance = eulerAnglesCovarianceSlot.captured
        val accelerometerCovariance = accelerometerCovarianceSlot.captured

        assertEquals(nedAttitude, processor.nedAttitude)
        assertEquals(eulerAngles, processor.eulerAngles)
        assertEquals(nedAttitudeCovariance, processor.nedAttitudeCovariance)
        assertEquals(eulerAnglesCovariance, processor.eulerAnglesCovariance)
        assertEquals(accelerometerCovariance, processor.accelerometerCovariance)
        assertNull(processor.rotationAxisCovariance)
    }

    @Test
    fun process_whenSeparateMeasurement_returnsFalse() {
        val processor = KalmanRelativeAttitudeProcessor()

        val previousTimestamp1: Long? = processor.getPrivateProperty("previousTimestamp")
        assertEquals(-1L, previousTimestamp1)

        val timestamp = System.nanoTime()
        val accelerometerMeasurement = createAccelerometerMeasurement(timestamp)
        val gyroscopeMeasurement = createGyroscopeMeasurement(timestamp)
        assertFalse(processor.process(accelerometerMeasurement, gyroscopeMeasurement))

        // check expected updates
        val previousTimestamp2: Long? = processor.getPrivateProperty("previousTimestamp")
        assertEquals(timestamp, previousTimestamp2)
    }

    @Test
    fun reset_setsInitialValues() {
        val processor = KalmanRelativeAttitudeProcessor()

        // setup
        val gravityProcessor: AccelerometerGravityProcessor? =
            processor.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val gravityProcessorSpy = spyk(gravityProcessor)
        processor.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        processor.setPrivateProperty("previousTimestamp", 1L)
        processor.setPrivateProperty("measurementsTimeIntervalSeconds", 1.0)
        processor.setPrivateProperty("initialized", true)

        val randomizer = UniformRandomizer()
        val q: Quaternion? = processor.getPrivateProperty("q")
        requireNotNull(q)
        q.setFromEulerAngles(
            randomizer.nextDouble(),
            randomizer.nextDouble(),
            randomizer.nextDouble()
        )

        val kalmanFilter: KalmanFilter? = processor.getPrivateProperty("kalmanFilter")
        requireNotNull(kalmanFilter)
        val statePre = kalmanFilter.statePre
        Matrix.fillWithUniformRandomValues(0.0, 1.0, statePre)

        val statePost = kalmanFilter.statePost
        Matrix.fillWithUniformRandomValues(0.0, 1.0, statePost)

        val errorCovPre = kalmanFilter.errorCovPre
        Matrix.fillWithUniformRandomValues(0.0, 1.0, errorCovPre)

        val errorCovPost = kalmanFilter.errorCovPost
        Matrix.fillWithUniformRandomValues(0.0, 1.0, errorCovPost)

        // reset
        processor.reset()

        // check
        verify(exactly = 1) { gravityProcessorSpy.reset() }

        val previousTimestamp: Long? = processor.getPrivateProperty("previousTimestamp")
        requireNotNull(previousTimestamp)
        assertEquals(-1L, previousTimestamp)

        val measurementsTimeIntervalSeconds: Double? = processor.getPrivateProperty("measurementsTimeIntervalSeconds")
        requireNotNull(measurementsTimeIntervalSeconds)
        assertEquals(0.0, measurementsTimeIntervalSeconds, 0.0)

        val initialized: Boolean? = processor.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertFalse(initialized)

        assertEquals(Quaternion(), q)

        val expectedState = Matrix(10, 1)
        expectedState.setElementAtIndex(3, 1.0)
        assertEquals(expectedState, statePre)
        assertEquals(expectedState, statePost)

        val expectedCov = Matrix(10, 10)
        assertEquals(expectedCov, errorCovPre)
        assertEquals(expectedCov, errorCovPost)
    }

    @Test
    fun processCovariance_whenErrorCovarianceIsNotPositiveDefinite_notifies() {
        justRun { numericalInstabilitiesListener.onNumericalInstabilityDetected(any()) }

        val processor = KalmanRelativeAttitudeProcessor(
            numericalInstabilitiesListener = numericalInstabilitiesListener
        )

        val kalmanFilter: KalmanFilter? = processor.getPrivateProperty("kalmanFilter")
        requireNotNull(kalmanFilter)

        val errorCovPre = kalmanFilter.errorCovPre
        Matrix.fillWithUniformRandomValues(0.0, 1.0, errorCovPre)

        val errorCovPost = kalmanFilter.errorCovPost
        Matrix.fillWithUniformRandomValues(0.0, 1.0, errorCovPost)

        processor.callPrivateFunc("processCovariance", false)

        verify(exactly = 1) { numericalInstabilitiesListener.onNumericalInstabilityDetected(processor) }
    }

    private fun getLocation(): Location {
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES)
        val longitudeDegrees =
            randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES)
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

        private fun createZeroAccelerometerMeasurement(timestamp: Long): AccelerometerSensorMeasurement {
            return AccelerometerSensorMeasurement(
                0.0f,
                0.0f,
                0.0f,
                null,
                null,
                null,
                timestamp,
                SensorAccuracy.HIGH,
                AccelerometerSensorType.ACCELEROMETER,
                SensorCoordinateSystem.NED
            )
        }

        private fun createAccelerometerMeasurement(timestamp: Long): AccelerometerSensorMeasurement {
            val randomizer = UniformRandomizer()
            val ax = randomizer.nextFloat()
            val ay = randomizer.nextFloat()
            val az = randomizer.nextFloat()
            val bx = randomizer.nextFloat()
            val by = randomizer.nextFloat()
            val bz = randomizer.nextFloat()
            return AccelerometerSensorMeasurement(
                ax,
                ay,
                az,
                bx,
                by,
                bz,
                timestamp,
                SensorAccuracy.HIGH,
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
                SensorCoordinateSystem.ENU
            )
        }

        private fun createGyroscopeMeasurement(timestamp: Long): GyroscopeSensorMeasurement {
            val randomizer = UniformRandomizer()
            val wx = randomizer.nextFloat()
            val wy = randomizer.nextFloat()
            val wz = randomizer.nextFloat()
            val bx = randomizer.nextFloat()
            val by = randomizer.nextFloat()
            val bz = randomizer.nextFloat()
            return GyroscopeSensorMeasurement(
                wx,
                wy,
                wz,
                bx,
                by,
                bz,
                timestamp,
                SensorAccuracy.HIGH,
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
                SensorCoordinateSystem.ENU
            )
        }
    }
}