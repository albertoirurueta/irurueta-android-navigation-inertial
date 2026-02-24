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

package com.irurueta.android.navigation.inertial.estimators.pose

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorManager
import android.location.Location
import android.os.SystemClock
import com.irurueta.android.navigation.inertial.collectors.AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorCollector
import com.irurueta.android.navigation.inertial.collectors.AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector
import com.irurueta.android.navigation.inertial.collectors.AttitudeAccelerometerAndGyroscopeSyncedSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorType
import com.irurueta.android.navigation.inertial.processors.attitude.BaseDoubleFusedGeomagneticAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.attitude.BaseFusedGeomagneticAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.filters.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.processors.filters.MedianAveragingFilter
import com.irurueta.android.navigation.inertial.processors.pose.AccelerometerDoubleFusedECEFAbsolutePoseProcessor
import com.irurueta.android.navigation.inertial.processors.pose.AccelerometerFusedECEFAbsolutePoseProcessor
import com.irurueta.android.navigation.inertial.processors.pose.AttitudeECEFAbsolutePoseProcessor
import com.irurueta.android.navigation.inertial.processors.pose.DoubleFusedECEFAbsolutePoseProcessor
import com.irurueta.android.navigation.inertial.processors.pose.FusedECEFAbsolutePoseProcessor
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.geometry.EuclideanTransformation3D
import com.irurueta.navigation.frames.ECEFFrame
import com.irurueta.navigation.frames.NEDVelocity
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

class EcefAbsolutePoseEstimatorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var poseAvailableListener: EcefAbsolutePoseEstimator.OnPoseAvailableListener

    @MockK(relaxUnitFun = true)
    private lateinit var accuracyChangedListener:
            EcefAbsolutePoseEstimator.OnAccuracyChangedListener

    @MockK
    private lateinit var location: Location

    @MockK
    private lateinit var context: Context

    @MockK
    private lateinit var sensorManager: SensorManager

    @MockK
    private lateinit var accelerometerSensor: Sensor

    @MockK
    private lateinit var gravitySensor: Sensor

    @MockK
    private lateinit var gyroscopeSensor: Sensor

    @MockK
    private lateinit var magnetometerSensor: Sensor

    @MockK
    private lateinit var attitudeSensor: Sensor

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, initialLocation)

        // check
        assertSame(context, estimator.context)
        assertSame(initialLocation, estimator.initialLocation)
        assertEquals(NEDVelocity(), estimator.initialVelocity)
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertTrue(estimator.useAttitudeSensor)
        assertFalse(estimator.useAccelerometerForAttitudeEstimation)
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
        assertEquals(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED, estimator.gyroscopeSensorType)
        assertNull(estimator.worldMagneticModel)
        assertNull(estimator.timestamp)
        assertFalse(estimator.useWorldMagneticModel)
        assertTrue(estimator.useAccurateLevelingProcessor)
        assertTrue(estimator.useAccurateRelativeGyroscopeAttitudeProcessor)
        assertTrue(estimator.useDoubleFusedAttitudeProcessor)
        assertFalse(estimator.estimatePoseTransformation)
        assertNull(estimator.poseAvailableListener)
        assertNull(estimator.accuracyChangedListener)
        assertTrue(estimator.adjustGravityNorm)
    }

    @Test
    fun constructor_whenAllProperties_setsExpectedValues() {
        val initialLocation = getLocation()
        val initialVelocity = NEDVelocity()
        val accelerometerAveragingFilter =
            MedianAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()
        val worldMagneticModel = WorldMagneticModel()
        val timestamp = Date()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            initialVelocity,
            SensorDelay.NORMAL,
            useAttitudeSensor = false,
            useAccelerometerForAttitudeEstimation = true,
            AccelerometerSensorType.ACCELEROMETER,
            MagnetometerSensorType.MAGNETOMETER,
            accelerometerAveragingFilter,
            GyroscopeSensorType.GYROSCOPE,
            worldMagneticModel,
            timestamp,
            useWorldMagneticModel = true,
            useAccurateLevelingProcessor = false,
            useAccurateRelativeGyroscopeAttitudeProcessor = false,
            useDoubleFusedAttitudeProcessor = false,
            estimatePoseTransformation = true,
            poseAvailableListener,
            accuracyChangedListener,
            adjustGravityNorm = false
        )

        // check
        assertSame(context, estimator.context)
        assertSame(initialLocation, estimator.initialLocation)
        assertSame(initialVelocity, estimator.initialVelocity)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertFalse(estimator.useAttitudeSensor)
        assertTrue(estimator.useAccelerometerForAttitudeEstimation)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            estimator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            estimator.magnetometerSensorType
        )
        assertSame(accelerometerAveragingFilter, estimator.accelerometerAveragingFilter)
        assertEquals(GyroscopeSensorType.GYROSCOPE, estimator.gyroscopeSensorType)
        assertSame(worldMagneticModel, estimator.worldMagneticModel)
        assertEquals(timestamp, estimator.timestamp)
        assertTrue(estimator.useWorldMagneticModel)
        assertFalse(estimator.useAccurateLevelingProcessor)
        assertFalse(estimator.useAccurateRelativeGyroscopeAttitudeProcessor)
        assertFalse(estimator.useDoubleFusedAttitudeProcessor)
        assertTrue(estimator.estimatePoseTransformation)
        assertSame(poseAvailableListener, estimator.poseAvailableListener)
        assertSame(accuracyChangedListener, estimator.accuracyChangedListener)
        assertFalse(estimator.adjustGravityNorm)
    }

    @Test
    fun initialLocation_whenUseAttitudeSensor_returnsExpectedValue() {
        val initialLocation = getLocation()
        val estimator =
            EcefAbsolutePoseEstimator(context, initialLocation, useAttitudeSensor = true)

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertSame(initialLocation, estimator.initialLocation)

        verify(exactly = 1) { attitudeProcessorSpy.initialLocation }
        verify { fusedProcessorSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }

        assertSame(initialLocation, attitudeProcessor.initialLocation)
        assertSame(initialLocation, fusedProcessor.initialLocation)
        assertSame(initialLocation, accelerometerFusedProcessor.initialLocation)
        assertSame(initialLocation, doubleFusedProcessor.initialLocation)
        assertSame(initialLocation, accelerometerDoubleFusedProcessor.initialLocation)
    }

    @Test
    fun initialLocation_whenNotUseAttitudeSensorUseDoubleFusedAttitudeProcessorAndUseAccelerometerForAttitudeEstimation_returnsExpectedValue() {
        val initialLocation = getLocation()
        val estimator =
            EcefAbsolutePoseEstimator(
                context,
                initialLocation,
                useAttitudeSensor = false,
                useDoubleFusedAttitudeProcessor = true,
                useAccelerometerForAttitudeEstimation = true
            )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertSame(initialLocation, estimator.initialLocation)

        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.initialLocation }
        verify { fusedProcessorSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { attitudeProcessorSpy wasNot Called }

        assertSame(initialLocation, attitudeProcessor.initialLocation)
        assertSame(initialLocation, fusedProcessor.initialLocation)
        assertSame(initialLocation, accelerometerFusedProcessor.initialLocation)
        assertSame(initialLocation, doubleFusedProcessor.initialLocation)
        assertSame(initialLocation, accelerometerDoubleFusedProcessor.initialLocation)
    }

    @Test
    fun initialLocation_whenNotUseAttitudeSensorUseDoubleFusedAttitudeProcessorAndNotUseAccelerometerForAttitudeEstimation_returnsExpectedValue() {
        val initialLocation = getLocation()
        val estimator =
            EcefAbsolutePoseEstimator(
                context,
                initialLocation,
                useAttitudeSensor = false,
                useDoubleFusedAttitudeProcessor = true,
                useAccelerometerForAttitudeEstimation = false
            )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertSame(initialLocation, estimator.initialLocation)

        verify(exactly = 1) { doubleFusedProcessorSpy.initialLocation }
        verify { fusedProcessorSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
        verify { attitudeProcessorSpy wasNot Called }

        assertSame(initialLocation, attitudeProcessor.initialLocation)
        assertSame(initialLocation, fusedProcessor.initialLocation)
        assertSame(initialLocation, accelerometerFusedProcessor.initialLocation)
        assertSame(initialLocation, doubleFusedProcessor.initialLocation)
        assertSame(initialLocation, accelerometerDoubleFusedProcessor.initialLocation)
    }

    @Test
    fun initialLocation_whenNotUseAttitudeSensorNotUseDoubleFusedAttitudeProcessorAndUseAccelerometerForAttitudeEstimation_returnsExpectedValue() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useAttitudeSensor = false,
            useDoubleFusedAttitudeProcessor = false,
            useAccelerometerForAttitudeEstimation = true
        )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertSame(initialLocation, estimator.initialLocation)

        verify(exactly = 1) { accelerometerFusedProcessorSpy.initialLocation }
        verify { fusedProcessorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
        verify { attitudeProcessorSpy wasNot Called }

        assertSame(initialLocation, attitudeProcessor.initialLocation)
        assertSame(initialLocation, fusedProcessor.initialLocation)
        assertSame(initialLocation, accelerometerFusedProcessor.initialLocation)
        assertSame(initialLocation, doubleFusedProcessor.initialLocation)
        assertSame(initialLocation, accelerometerDoubleFusedProcessor.initialLocation)
    }

    @Test
    fun initialLocation_whenNotUseAttitudeSensorNotUseDoubleFusedAttitudeProcessorAndNotUseAccelerometerForAttitudeEstimation_returnsExpectedValue() {
        val initialLocation = getLocation()
        val estimator =
            EcefAbsolutePoseEstimator(
                context,
                initialLocation,
                useAttitudeSensor = false,
                useDoubleFusedAttitudeProcessor = false,
                useAccelerometerForAttitudeEstimation = false
            )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertSame(initialLocation, estimator.initialLocation)

        verify(exactly = 1) { fusedProcessorSpy.initialLocation }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
        verify { attitudeProcessorSpy wasNot Called }

        assertSame(initialLocation, attitudeProcessor.initialLocation)
        assertSame(initialLocation, fusedProcessor.initialLocation)
        assertSame(initialLocation, accelerometerFusedProcessor.initialLocation)
        assertSame(initialLocation, doubleFusedProcessor.initialLocation)
        assertSame(initialLocation, accelerometerDoubleFusedProcessor.initialLocation)
    }

    @Test
    fun initialVelocity_whenUseAttitudeSensor_returnsExpectedValue() {
        val initialLocation = getLocation()
        val initialVelocity = NEDVelocity()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            initialVelocity,
            useAttitudeSensor = true
        )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertSame(initialVelocity, estimator.initialVelocity)

        verify(exactly = 1) { attitudeProcessorSpy.initialVelocity }
        verify { fusedProcessorSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }

        assertSame(initialVelocity, attitudeProcessor.initialVelocity)
        assertSame(initialVelocity, fusedProcessor.initialVelocity)
        assertSame(initialVelocity, accelerometerFusedProcessor.initialVelocity)
        assertSame(initialVelocity, doubleFusedProcessor.initialVelocity)
        assertSame(initialVelocity, accelerometerDoubleFusedProcessor.initialVelocity)
    }

    @Test
    fun initialVelocity_whenNotUseAttitudeSensorUseDoubleFusedAttitudeProcessorAndUseAccelerometerForAttitudeEstimation_returnsExpectedValue() {
        val initialLocation = getLocation()
        val initialVelocity = NEDVelocity()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            initialVelocity,
            useAttitudeSensor = false,
            useDoubleFusedAttitudeProcessor = true,
            useAccelerometerForAttitudeEstimation = true
        )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertSame(initialVelocity, estimator.initialVelocity)

        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.initialVelocity }
        verify { fusedProcessorSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { attitudeProcessorSpy wasNot Called }

        assertSame(initialVelocity, attitudeProcessor.initialVelocity)
        assertSame(initialVelocity, fusedProcessor.initialVelocity)
        assertSame(initialVelocity, accelerometerFusedProcessor.initialVelocity)
        assertSame(initialVelocity, doubleFusedProcessor.initialVelocity)
        assertSame(initialVelocity, accelerometerDoubleFusedProcessor.initialVelocity)
    }

    @Test
    fun initialVelocity_whenNotUseAttitudeSensorUseDoubleFusedAttitudeProcessorAndNotUseAccelerometerForAttitudeEstimation_returnsExpectedValue() {
        val initialLocation = getLocation()
        val initialVelocity = NEDVelocity()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            initialVelocity,
            useAttitudeSensor = false,
            useDoubleFusedAttitudeProcessor = true,
            useAccelerometerForAttitudeEstimation = false
        )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertSame(initialVelocity, estimator.initialVelocity)

        verify(exactly = 1) { doubleFusedProcessorSpy.initialVelocity }
        verify { fusedProcessorSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
        verify { attitudeProcessorSpy wasNot Called }

        assertSame(initialVelocity, attitudeProcessor.initialVelocity)
        assertSame(initialVelocity, fusedProcessor.initialVelocity)
        assertSame(initialVelocity, accelerometerFusedProcessor.initialVelocity)
        assertSame(initialVelocity, doubleFusedProcessor.initialVelocity)
        assertSame(initialVelocity, accelerometerDoubleFusedProcessor.initialVelocity)
    }

    @Test
    fun initialVelocity_whenNotUseAttitudeSensorNotUseDoubleFusedAttitudeProcessorAndUseAccelerometerForAttitudeEstimation_returnsExpectedValue() {
        val initialLocation = getLocation()
        val initialVelocity = NEDVelocity()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            initialVelocity,
            useAttitudeSensor = false,
            useDoubleFusedAttitudeProcessor = false,
            useAccelerometerForAttitudeEstimation = true
        )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertSame(initialVelocity, estimator.initialVelocity)

        verify(exactly = 1) { accelerometerFusedProcessorSpy.initialVelocity }
        verify { fusedProcessorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
        verify { attitudeProcessorSpy wasNot Called }

        assertSame(initialVelocity, attitudeProcessor.initialVelocity)
        assertSame(initialVelocity, fusedProcessor.initialVelocity)
        assertSame(initialVelocity, accelerometerFusedProcessor.initialVelocity)
        assertSame(initialVelocity, doubleFusedProcessor.initialVelocity)
        assertSame(initialVelocity, accelerometerDoubleFusedProcessor.initialVelocity)
    }

    @Test
    fun initialVelocity_whenNotUseAttitudeSensorNotUseDoubleFusedAttitudeProcessorAndNotUseAccelerometerForAttitudeEstimation_returnsExpectedValue() {
        val initialLocation = getLocation()
        val initialVelocity = NEDVelocity()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            initialVelocity,
            useAttitudeSensor = false,
            useDoubleFusedAttitudeProcessor = false,
            useAccelerometerForAttitudeEstimation = false
        )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertSame(initialVelocity, estimator.initialVelocity)

        verify(exactly = 1) { fusedProcessorSpy.initialVelocity }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
        verify { attitudeProcessorSpy wasNot Called }

        assertSame(initialVelocity, attitudeProcessor.initialVelocity)
        assertSame(initialVelocity, fusedProcessor.initialVelocity)
        assertSame(initialVelocity, accelerometerFusedProcessor.initialVelocity)
        assertSame(initialVelocity, doubleFusedProcessor.initialVelocity)
        assertSame(initialVelocity, accelerometerDoubleFusedProcessor.initialVelocity)
    }

    @Test
    fun estimatePoseTransformation_whenUseAttitudeSensor_returnsExpectedValue() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useAttitudeSensor = true
        )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertFalse(estimator.estimatePoseTransformation)

        verify(exactly = 1) { attitudeProcessorSpy.estimatePoseTransformation }
        verify { fusedProcessorSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }

        assertFalse(attitudeProcessor.estimatePoseTransformation)
        assertFalse(fusedProcessor.estimatePoseTransformation)
        assertFalse(accelerometerFusedProcessor.estimatePoseTransformation)
        assertFalse(doubleFusedProcessor.estimatePoseTransformation)
        assertFalse(accelerometerDoubleFusedProcessor.estimatePoseTransformation)
    }

    @Test
    fun estimatePoseTransformation_whenNotUseAttitudeSensorUseDoubleFusedAttitudeProcessorAndUseAccelerometerForAttitudeEstimation_returnsExpectedValue() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useAttitudeSensor = false,
            useDoubleFusedAttitudeProcessor = true,
            useAccelerometerForAttitudeEstimation = true
        )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertFalse(estimator.estimatePoseTransformation)

        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.estimatePoseTransformation }
        verify { fusedProcessorSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { attitudeProcessorSpy wasNot Called }

        assertFalse(attitudeProcessor.estimatePoseTransformation)
        assertFalse(fusedProcessor.estimatePoseTransformation)
        assertFalse(accelerometerFusedProcessor.estimatePoseTransformation)
        assertFalse(doubleFusedProcessor.estimatePoseTransformation)
        assertFalse(accelerometerDoubleFusedProcessor.estimatePoseTransformation)
    }

    @Test
    fun estimatePoseTransformation_whenNotUseAttitudeSensorUseDoubleFusedAttitudeProcessorAndNotUseAccelerometerForAttitudeEstimation_returnsExpectedValue() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useAttitudeSensor = false,
            useDoubleFusedAttitudeProcessor = true,
            useAccelerometerForAttitudeEstimation = false
        )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertFalse(estimator.estimatePoseTransformation)

        verify(exactly = 1) { doubleFusedProcessorSpy.estimatePoseTransformation }
        verify { fusedProcessorSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
        verify { attitudeProcessorSpy wasNot Called }

        assertFalse(attitudeProcessor.estimatePoseTransformation)
        assertFalse(fusedProcessor.estimatePoseTransformation)
        assertFalse(accelerometerFusedProcessor.estimatePoseTransformation)
        assertFalse(doubleFusedProcessor.estimatePoseTransformation)
        assertFalse(accelerometerDoubleFusedProcessor.estimatePoseTransformation)
    }

    @Test
    fun estimatePoseTransformation_whenNotUseAttitudeSensorNotUseDoubleFusedAttitudeProcessorAndUseAccelerometerForAttitudeEstimation_returnsExpectedValue() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useAttitudeSensor = false,
            useDoubleFusedAttitudeProcessor = false,
            useAccelerometerForAttitudeEstimation = true
        )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertFalse(estimator.estimatePoseTransformation)

        verify(exactly = 1) { accelerometerFusedProcessorSpy.estimatePoseTransformation }
        verify { fusedProcessorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
        verify { attitudeProcessorSpy wasNot Called }

        assertFalse(attitudeProcessor.estimatePoseTransformation)
        assertFalse(fusedProcessor.estimatePoseTransformation)
        assertFalse(accelerometerFusedProcessor.estimatePoseTransformation)
        assertFalse(doubleFusedProcessor.estimatePoseTransformation)
        assertFalse(accelerometerDoubleFusedProcessor.estimatePoseTransformation)
    }

    @Test
    fun estimatePoseTransformation_whenNotUseAttitudeSensorNotUseDoubleFusedAttitudeProcessorAndNotUseAccelerometerForAttitudeEstimation_returnsExpectedValue() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useAttitudeSensor = false,
            useDoubleFusedAttitudeProcessor = false,
            useAccelerometerForAttitudeEstimation = false
        )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertFalse(estimator.estimatePoseTransformation)

        verify(exactly = 1) { fusedProcessorSpy.estimatePoseTransformation }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
        verify { attitudeProcessorSpy wasNot Called }

        assertFalse(attitudeProcessor.estimatePoseTransformation)
        assertFalse(fusedProcessor.estimatePoseTransformation)
        assertFalse(accelerometerFusedProcessor.estimatePoseTransformation)
        assertFalse(doubleFusedProcessor.estimatePoseTransformation)
        assertFalse(accelerometerDoubleFusedProcessor.estimatePoseTransformation)
    }

    @Test
    fun useAccurateLevelingProcessor_returnsExpectedValue() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useAccurateLevelingProcessor = true
        )

        // check default value
        assertTrue(estimator.useAccurateLevelingProcessor)

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        assertTrue(fusedProcessor.useAccurateLevelingProcessor)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        assertTrue(accelerometerFusedProcessor.useAccurateLevelingProcessor)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        assertTrue(doubleFusedProcessor.useAccurateLevelingProcessor)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        assertTrue(accelerometerDoubleFusedProcessor.useAccurateLevelingProcessor)
    }

    @Test
    fun worldMagneticModel_returnsExpectedValue() {
        val initialLocation = getLocation()
        val worldMagneticModel = WorldMagneticModel()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            worldMagneticModel = worldMagneticModel
        )

        // check default value
        assertSame(worldMagneticModel, estimator.worldMagneticModel)

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        assertSame(worldMagneticModel, fusedProcessor.worldMagneticModel)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        assertSame(worldMagneticModel, accelerometerFusedProcessor.worldMagneticModel)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        assertSame(worldMagneticModel, doubleFusedProcessor.worldMagneticModel)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        assertSame(worldMagneticModel, accelerometerDoubleFusedProcessor.worldMagneticModel)
    }

    @Test
    fun useWorldMagneticModel_returnsExpectedValue() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useWorldMagneticModel = true
        )

        // check default value
        assertTrue(estimator.useWorldMagneticModel)

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        assertTrue(fusedProcessor.useWorldMagneticModel)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        assertTrue(accelerometerFusedProcessor.useWorldMagneticModel)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        assertTrue(doubleFusedProcessor.useWorldMagneticModel)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        assertTrue(accelerometerDoubleFusedProcessor.useWorldMagneticModel)
    }

    @Test
    fun timestamp_setsExpectedValue() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation
        )

        // check default value
        assertNull(estimator.timestamp)

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        assertNull(fusedProcessor.currentDate)
        assertNull(accelerometerFusedProcessor.currentDate)
        assertNull(doubleFusedProcessor.currentDate)
        assertNull(accelerometerDoubleFusedProcessor.currentDate)

        // set new value
        val timestamp = Date()
        estimator.timestamp = timestamp

        // check
        assertSame(timestamp, estimator.timestamp)
        verify(exactly = 1) { fusedProcessorSpy.currentDate = timestamp }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.currentDate = timestamp }
        verify(exactly = 1) { doubleFusedProcessorSpy.currentDate = timestamp }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.currentDate = timestamp }
    }

    @Test
    fun useAccurateRelativeGyroscopeAttitudeProcessor_returnsExpectedValue() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation
        )

        // check default value
        assertTrue(estimator.useAccurateRelativeGyroscopeAttitudeProcessor)

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        assertTrue(fusedProcessor.useAccurateRelativeGyroscopeAttitudeProcessor)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        assertTrue(accelerometerFusedProcessor.useAccurateRelativeGyroscopeAttitudeProcessor)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        assertTrue(doubleFusedProcessor.useAccurateRelativeGyroscopeAttitudeProcessor)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        assertTrue(accelerometerDoubleFusedProcessor.useAccurateRelativeGyroscopeAttitudeProcessor)
    }

    @Test
    fun useIndirectAttitudeInterpolation_setsExpectedValue() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation
        )

        // check default value
        assertTrue(estimator.useIndirectAttitudeInterpolation)

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        assertTrue(fusedProcessor.useIndirectAttitudeInterpolation)
        assertTrue(accelerometerFusedProcessor.useIndirectAttitudeInterpolation)
        assertTrue(doubleFusedProcessor.useIndirectAttitudeInterpolation)
        assertTrue(accelerometerDoubleFusedProcessor.useIndirectAttitudeInterpolation)

        // set new value
        estimator.useIndirectAttitudeInterpolation = false

        // check
        assertFalse(estimator.useIndirectAttitudeInterpolation)
        verify(exactly = 1) { fusedProcessorSpy.useIndirectAttitudeInterpolation = false }
        verify(exactly = 1) {
            accelerometerFusedProcessorSpy.useIndirectAttitudeInterpolation = false
        }
        verify(exactly = 1) { doubleFusedProcessorSpy.useIndirectAttitudeInterpolation = false }
        verify(exactly = 1) {
            accelerometerDoubleFusedProcessorSpy.useIndirectAttitudeInterpolation = false
        }
    }

    @Test
    fun attitudeInterpolationValue_setsExpectedValue() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation
        )

        // check default value
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            estimator.attitudeInterpolationValue,
            0.0
        )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            fusedProcessor.attitudeInterpolationValue,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            accelerometerFusedProcessor.attitudeInterpolationValue,
            0.0
        )
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            doubleFusedProcessor.attitudeInterpolationValue,
            0.0
        )
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            accelerometerDoubleFusedProcessor.attitudeInterpolationValue,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val attitudeInterpolationValue = randomizer.nextDouble()
        estimator.attitudeInterpolationValue = attitudeInterpolationValue

        // check
        assertEquals(attitudeInterpolationValue, estimator.attitudeInterpolationValue, 0.0)
        verify(exactly = 1) {
            fusedProcessorSpy.attitudeInterpolationValue = attitudeInterpolationValue
        }
        verify(exactly = 1) {
            accelerometerFusedProcessorSpy.attitudeInterpolationValue = attitudeInterpolationValue
        }
        verify(exactly = 1) {
            doubleFusedProcessorSpy.attitudeInterpolationValue = attitudeInterpolationValue
        }
        verify(exactly = 1) {
            accelerometerDoubleFusedProcessorSpy.attitudeInterpolationValue =
                attitudeInterpolationValue
        }
    }

    @Test
    fun attitudeIndirectInterpolationWeight_setsExpectedValue() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation
        )

        // check default value
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.attitudeIndirectInterpolationWeight,
            0.0
        )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            fusedProcessor.attitudeIndirectInterpolationWeight,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            accelerometerFusedProcessor.attitudeIndirectInterpolationWeight,
            0.0
        )
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            doubleFusedProcessor.attitudeIndirectInterpolationWeight,
            0.0
        )
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            accelerometerDoubleFusedProcessor.attitudeIndirectInterpolationWeight,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val attitudeIndirectInterpolationWeight = randomizer.nextDouble()
        estimator.attitudeIndirectInterpolationWeight = attitudeIndirectInterpolationWeight

        // check
        assertEquals(
            attitudeIndirectInterpolationWeight,
            estimator.attitudeIndirectInterpolationWeight,
            0.0
        )
        verify(exactly = 1) {
            fusedProcessorSpy.attitudeIndirectInterpolationWeight =
                attitudeIndirectInterpolationWeight
        }
        verify(exactly = 1) {
            accelerometerFusedProcessorSpy.attitudeIndirectInterpolationWeight =
                attitudeIndirectInterpolationWeight
        }
        verify(exactly = 1) {
            doubleFusedProcessorSpy.attitudeIndirectInterpolationWeight =
                attitudeIndirectInterpolationWeight
        }
        verify(exactly = 1) {
            accelerometerDoubleFusedProcessorSpy.attitudeIndirectInterpolationWeight =
                attitudeIndirectInterpolationWeight
        }
    }

    @Test
    fun attitudeOutlierThreshold_setsExpectedValue() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation
        )

        // check default value
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            estimator.attitudeOutlierThreshold,
            0.0
        )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            fusedProcessor.attitudeOutlierThreshold,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            accelerometerFusedProcessor.attitudeOutlierThreshold,
            0.0
        )
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            doubleFusedProcessor.attitudeOutlierThreshold,
            0.0
        )
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            accelerometerDoubleFusedProcessor.attitudeOutlierThreshold,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val attitudeOutlierThreshold = randomizer.nextDouble()
        estimator.attitudeOutlierThreshold = attitudeOutlierThreshold

        // check
        assertEquals(
            attitudeOutlierThreshold,
            estimator.attitudeOutlierThreshold,
            0.0
        )
        verify(exactly = 1) {
            fusedProcessorSpy.attitudeOutlierThreshold = attitudeOutlierThreshold
        }
        verify(exactly = 1) {
            accelerometerFusedProcessorSpy.attitudeOutlierThreshold = attitudeOutlierThreshold
        }
        verify(exactly = 1) {
            doubleFusedProcessorSpy.attitudeOutlierThreshold = attitudeOutlierThreshold
        }
        verify(exactly = 1) {
            accelerometerDoubleFusedProcessorSpy.attitudeOutlierThreshold = attitudeOutlierThreshold
        }
    }

    @Test
    fun attitudeOutlierPanicThreshold_setsExpectedValue() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation
        )

        // check default value
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.attitudeOutlierPanicThreshold,
            0.0
        )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            fusedProcessor.attitudeOutlierPanicThreshold,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            accelerometerFusedProcessor.attitudeOutlierPanicThreshold,
            0.0
        )
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            doubleFusedProcessor.attitudeOutlierPanicThreshold,
            0.0
        )
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            accelerometerDoubleFusedProcessor.attitudeOutlierPanicThreshold,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val attitudeOutlierPanicThreshold = randomizer.nextDouble()
        estimator.attitudeOutlierPanicThreshold = attitudeOutlierPanicThreshold

        // check
        assertEquals(
            attitudeOutlierPanicThreshold,
            estimator.attitudeOutlierPanicThreshold,
            0.0
        )
        verify(exactly = 1) {
            fusedProcessorSpy.attitudeOutlierPanicThreshold = attitudeOutlierPanicThreshold
        }
        verify(exactly = 1) {
            accelerometerFusedProcessorSpy.attitudeOutlierPanicThreshold =
                attitudeOutlierPanicThreshold
        }
        verify(exactly = 1) {
            doubleFusedProcessorSpy.attitudeOutlierPanicThreshold = attitudeOutlierPanicThreshold
        }
        verify(exactly = 1) {
            accelerometerDoubleFusedProcessorSpy.attitudeOutlierPanicThreshold =
                attitudeOutlierPanicThreshold
        }
    }

    @Test
    fun attitudePanicCounterThreshold_setsExpectedValue() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation
        )

        // check default value
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.attitudePanicCounterThreshold,
        )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            fusedProcessor.attitudePanicCounterThreshold,
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            accelerometerFusedProcessor.attitudePanicCounterThreshold,
        )
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            doubleFusedProcessor.attitudePanicCounterThreshold,
        )
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            accelerometerDoubleFusedProcessor.attitudePanicCounterThreshold,
        )

        // set new value
        val randomizer = UniformRandomizer()
        val attitudePanicCounterThreshold = randomizer.nextInt(1, 100)
        estimator.attitudePanicCounterThreshold = attitudePanicCounterThreshold

        // check
        assertEquals(attitudePanicCounterThreshold, estimator.attitudePanicCounterThreshold)
        verify(exactly = 1) {
            fusedProcessorSpy.attitudePanicCounterThreshold = attitudePanicCounterThreshold
        }
        verify(exactly = 1) {
            accelerometerFusedProcessorSpy.attitudePanicCounterThreshold =
                attitudePanicCounterThreshold
        }
        verify(exactly = 1) {
            doubleFusedProcessorSpy.attitudePanicCounterThreshold = attitudePanicCounterThreshold
        }
        verify(exactly = 1) {
            accelerometerDoubleFusedProcessorSpy.attitudePanicCounterThreshold =
                attitudePanicCounterThreshold
        }
    }

    @Test
    fun timeIntervalSeconds_whenUseAttitudeSensor_returnsNull() {
        val initialLocation = getLocation()
        val estimator =
            EcefAbsolutePoseEstimator(context, initialLocation, useAttitudeSensor = true)

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertNull(estimator.timeIntervalSeconds)

        verify { fusedProcessorSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
    }

    @Test
    fun timeIntervalSeconds_whenNotUseAttitudeSensorUseDoubleFusedAttitudeProcessorAndUseAccelerometerForAttitudeEstimation_returnsExpectedValue() {
        val initialLocation = getLocation()
        val estimator =
            EcefAbsolutePoseEstimator(
                context,
                initialLocation,
                useAttitudeSensor = false,
                useDoubleFusedAttitudeProcessor = true,
                useAccelerometerForAttitudeEstimation = true
            )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        every { accelerometerDoubleFusedProcessorSpy.gyroscopeTimeIntervalSeconds }.returns(
            TIME_INTERVAL
        )
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val timeIntervalSeconds = estimator.timeIntervalSeconds
        requireNotNull(timeIntervalSeconds)
        assertEquals(TIME_INTERVAL, timeIntervalSeconds, 0.0)

        verify { fusedProcessorSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.gyroscopeTimeIntervalSeconds }
    }

    @Test
    fun timeIntervalSeconds_whenNotUseAttitudeSensorUseDoubleFusedAttitudeProcessorAndNotUseAccelerometerForAttitudeEstimation_returnsExpectedValue() {
        val initialLocation = getLocation()
        val estimator =
            EcefAbsolutePoseEstimator(
                context,
                initialLocation,
                useAttitudeSensor = false,
                useDoubleFusedAttitudeProcessor = true,
                useAccelerometerForAttitudeEstimation = false
            )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        every { doubleFusedProcessorSpy.gyroscopeTimeIntervalSeconds }.returns(
            TIME_INTERVAL
        )
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val timeIntervalSeconds = estimator.timeIntervalSeconds
        requireNotNull(timeIntervalSeconds)
        assertEquals(TIME_INTERVAL, timeIntervalSeconds, 0.0)

        verify { fusedProcessorSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
        verify(exactly = 1) { doubleFusedProcessorSpy.gyroscopeTimeIntervalSeconds }
    }

    @Test
    fun timeIntervalSeconds_whenNotUseAttitudeSensorNotUseDoubleFusedAttitudeProcessorAndUseAccelerometerForAttitudeEstimation_returnsExpectedValue() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useAttitudeSensor = false,
            useDoubleFusedAttitudeProcessor = false,
            useAccelerometerForAttitudeEstimation = true
        )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        every { accelerometerFusedProcessorSpy.gyroscopeTimeIntervalSeconds }.returns(
            TIME_INTERVAL
        )
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val timeIntervalSeconds = estimator.timeIntervalSeconds
        requireNotNull(timeIntervalSeconds)
        assertEquals(
            TIME_INTERVAL,
            timeIntervalSeconds,
            0.0
        )

        verify { fusedProcessorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.gyroscopeTimeIntervalSeconds }
    }

    @Test
    fun timeIntervalSeconds_whenNotUseAttitudeSensorNotUseDoubleFusedAttitudeProcessorAndNotUseAccelerometerForAttitudeEstimation_returnsExpectedValue() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useAttitudeSensor = false,
            useDoubleFusedAttitudeProcessor = false,
            useAccelerometerForAttitudeEstimation = false
        )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        every { fusedProcessorSpy.gyroscopeTimeIntervalSeconds }.returns(
            TIME_INTERVAL
        )
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val timeIntervalSeconds = estimator.timeIntervalSeconds
        requireNotNull(timeIntervalSeconds)
        assertEquals(
            TIME_INTERVAL,
            timeIntervalSeconds,
            0.0
        )

        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
        verify(exactly = 1) { fusedProcessorSpy.gyroscopeTimeIntervalSeconds }
    }

    @Test
    fun useLeveledRelativeAttitudeRespectStart_setsExpectedValue() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation
        )

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // check default value
        assertTrue(estimator.useLeveledRelativeAttitudeRespectStart)

        assertTrue(fusedProcessor.useLeveledRelativeAttitudeRespectStart)
        assertTrue(accelerometerFusedProcessor.useLeveledRelativeAttitudeRespectStart)
        assertTrue(doubleFusedProcessor.useLeveledRelativeAttitudeRespectStart)
        assertTrue(accelerometerDoubleFusedProcessor.useLeveledRelativeAttitudeRespectStart)
        assertTrue(attitudeProcessor.useLeveledRelativeAttitudeRespectStart)

        // set new value
        estimator.useLeveledRelativeAttitudeRespectStart = false

        // check
        assertFalse(estimator.useLeveledRelativeAttitudeRespectStart)

        verify(exactly = 1) { fusedProcessorSpy.useLeveledRelativeAttitudeRespectStart = false }
        verify(exactly = 1) {
            accelerometerFusedProcessorSpy.useLeveledRelativeAttitudeRespectStart = false
        }
        verify(exactly = 1) {
            doubleFusedProcessorSpy.useLeveledRelativeAttitudeRespectStart = false
        }
        verify(exactly = 1) {
            accelerometerDoubleFusedProcessorSpy.useLeveledRelativeAttitudeRespectStart = false
        }
        verify(exactly = 1) { attitudeProcessorSpy.useLeveledRelativeAttitudeRespectStart = false }
    }

    @Test
    fun adjustGravityNorm_whenNotRunning_setsExpectedValue() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, initialLocation)

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)

        // check default value
        assertFalse(estimator.running)
        assertTrue(estimator.adjustGravityNorm)
        assertTrue(fusedProcessor.adjustGravityNorm)
        assertTrue(accelerometerFusedProcessor.adjustGravityNorm)
        assertTrue(doubleFusedProcessor.adjustGravityNorm)
        assertTrue(accelerometerDoubleFusedProcessor.adjustGravityNorm)

        // set new value
        estimator.adjustGravityNorm = false

        // check
        assertFalse(estimator.adjustGravityNorm)
        assertFalse(fusedProcessor.adjustGravityNorm)
        assertFalse(accelerometerFusedProcessor.adjustGravityNorm)
        assertFalse(doubleFusedProcessor.adjustGravityNorm)
        assertFalse(accelerometerDoubleFusedProcessor.adjustGravityNorm)
    }

    @Test
    fun adjustGravityNorm_whenRunning_throwsIllegalStateException() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, initialLocation)

        assertFalse(estimator.running)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        assertThrows(IllegalStateException::class.java) {
            estimator.adjustGravityNorm = false
        }
    }

    @Test
    fun start_whenRunning_throwsIllegalStateException() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            val initialLocation = getLocation()
            val estimator = EcefAbsolutePoseEstimator(
                context,
                initialLocation
            )

            assertFalse(estimator.running)

            // set as running
            estimator.setPrivateProperty("running", true)

            assertTrue(estimator.running)

            assertThrows(IllegalStateException::class.java) {
                estimator.start()
            }
        }
    }

    @Test
    fun start_whenUseAttitudeSensor_resetsProcessorAndStartsSyncer() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useAttitudeSensor = true
        )

        val startTimestamp = System.nanoTime()

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val fusedCollector: AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val fusedCollectorSpy = spyk(fusedCollector)
        estimator.setPrivateProperty("fusedCollector", fusedCollectorSpy)

        val accelerometerFusedCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val accelerometerFusedCollectorSpy = spyk(accelerometerFusedCollector)
        estimator.setPrivateProperty("accelerometerFusedCollector", accelerometerFusedCollectorSpy)

        val attitudeCollector: AttitudeAccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("attitudeCollector")
        requireNotNull(attitudeCollector)
        val attitudeCollectorSpy = spyk(attitudeCollector)
        every { attitudeCollectorSpy.start(startTimestamp) }.returns(true)
        estimator.setPrivateProperty("attitudeCollector", attitudeCollectorSpy)

        assertTrue(estimator.start(startTimestamp))

        verify { attitudeProcessorSpy.reset() }
        verify(exactly = 1) { attitudeCollectorSpy.start(startTimestamp) }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { accelerometerFusedCollectorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { fusedProcessorSpy wasNot Called }
        verify { fusedCollectorSpy wasNot Called }
    }

    @Test
    fun start_whenNotUseAttitudeSensorUseAccelerometerForAttitudeEstimationAndUseDoubleFusedAttitudeProcessor_resetsProcessorAndStartsSyncer() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useAttitudeSensor = false,
            useAccelerometerForAttitudeEstimation = true,
            useDoubleFusedAttitudeProcessor = true
        )

        val startTimestamp = System.nanoTime()

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val fusedCollector: AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val fusedCollectorSpy = spyk(fusedCollector)
        estimator.setPrivateProperty("fusedCollector", fusedCollectorSpy)

        val accelerometerFusedCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val accelerometerFusedCollectorSpy = spyk(accelerometerFusedCollector)
        every { accelerometerFusedCollectorSpy.start(startTimestamp) }.returns(true)
        estimator.setPrivateProperty("accelerometerFusedCollector", accelerometerFusedCollectorSpy)

        val attitudeCollector: AttitudeAccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("attitudeCollector")
        requireNotNull(attitudeCollector)
        val attitudeCollectorSpy = spyk(attitudeCollector)
        estimator.setPrivateProperty("attitudeCollector", attitudeCollectorSpy)

        assertTrue(estimator.start(startTimestamp))

        verify { accelerometerDoubleFusedProcessorSpy.reset() }
        verify(exactly = 1) { accelerometerFusedCollectorSpy.start(startTimestamp) }
        verify { attitudeProcessorSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { attitudeCollectorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { fusedProcessorSpy wasNot Called }
        verify { fusedCollectorSpy wasNot Called }
    }

    @Test
    fun start_whenNotUseAttitudeSensorUseAccelerometerForAttitudeEstimationAndNotUseDoubleFusedAttitudeProcessor_resetsProcessorAndStartsSyncer() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useAttitudeSensor = false,
            useAccelerometerForAttitudeEstimation = true,
            useDoubleFusedAttitudeProcessor = false
        )

        val startTimestamp = System.nanoTime()

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val fusedCollector: AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val fusedCollectorSpy = spyk(fusedCollector)
        estimator.setPrivateProperty("fusedCollector", fusedCollectorSpy)

        val accelerometerFusedCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val accelerometerFusedCollectorSpy = spyk(accelerometerFusedCollector)
        every { accelerometerFusedCollectorSpy.start(startTimestamp) }.returns(true)
        estimator.setPrivateProperty("accelerometerFusedCollector", accelerometerFusedCollectorSpy)

        val attitudeCollector: AttitudeAccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("attitudeCollector")
        requireNotNull(attitudeCollector)
        val attitudeCollectorSpy = spyk(attitudeCollector)
        estimator.setPrivateProperty("attitudeCollector", attitudeCollectorSpy)

        assertTrue(estimator.start(startTimestamp))

        verify { accelerometerFusedProcessorSpy.reset() }
        verify(exactly = 1) { accelerometerFusedCollectorSpy.start(startTimestamp) }
        verify { attitudeProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
        verify { attitudeCollectorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { fusedProcessorSpy wasNot Called }
        verify { fusedCollectorSpy wasNot Called }
    }

    @Test
    fun start_whenNotUseAttitudeSensorNotUseAccelerometerForAttitudeEstimationAndUseDoubleFusedAttitudeProcessor_resetsProcessorAndStartsSyncer() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useAttitudeSensor = false,
            useAccelerometerForAttitudeEstimation = false,
            useDoubleFusedAttitudeProcessor = true
        )

        val startTimestamp = System.nanoTime()

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val fusedCollector: AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val fusedCollectorSpy = spyk(fusedCollector)
        every { fusedCollectorSpy.start(startTimestamp) }.returns(true)
        estimator.setPrivateProperty("fusedCollector", fusedCollectorSpy)

        val accelerometerFusedCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val accelerometerFusedCollectorSpy = spyk(accelerometerFusedCollector)
        estimator.setPrivateProperty("accelerometerFusedCollector", accelerometerFusedCollectorSpy)

        val attitudeCollector: AttitudeAccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("attitudeCollector")
        requireNotNull(attitudeCollector)
        val attitudeCollectorSpy = spyk(attitudeCollector)
        estimator.setPrivateProperty("attitudeCollector", attitudeCollectorSpy)

        assertTrue(estimator.start(startTimestamp))

        verify { doubleFusedProcessorSpy.reset() }
        verify(exactly = 1) { fusedCollectorSpy.start(startTimestamp) }
        verify { attitudeProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
        verify { attitudeCollectorSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { fusedProcessorSpy wasNot Called }
        verify { accelerometerFusedCollectorSpy wasNot Called }
    }

    @Test
    fun start_whenNotUseAttitudeSensorNotUseAccelerometerForAttitudeEstimationAndNotUseDoubleFusedAttitudeProcessor_resetsProcessorAndStartsSyncer() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useAttitudeSensor = false,
            useAccelerometerForAttitudeEstimation = false,
            useDoubleFusedAttitudeProcessor = false
        )

        val startTimestamp = System.nanoTime()

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val fusedCollector: AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val fusedCollectorSpy = spyk(fusedCollector)
        every { fusedCollectorSpy.start(startTimestamp) }.returns(true)
        estimator.setPrivateProperty("fusedCollector", fusedCollectorSpy)

        val accelerometerFusedCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val accelerometerFusedCollectorSpy = spyk(accelerometerFusedCollector)
        estimator.setPrivateProperty("accelerometerFusedCollector", accelerometerFusedCollectorSpy)

        val attitudeCollector: AttitudeAccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("attitudeCollector")
        requireNotNull(attitudeCollector)
        val attitudeCollectorSpy = spyk(attitudeCollector)
        estimator.setPrivateProperty("attitudeCollector", attitudeCollectorSpy)

        assertTrue(estimator.start(startTimestamp))

        verify { fusedProcessorSpy.reset() }
        verify(exactly = 1) { fusedCollectorSpy.start(startTimestamp) }
        verify { attitudeProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
        verify { attitudeCollectorSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { accelerometerFusedCollectorSpy wasNot Called }
    }

    @Test
    fun stop_stopsAllSyncers() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }
            .returns(accelerometerSensor)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }
            .returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }
            .returns(gyroscopeSensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }
            .returns(magnetometerSensor)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }
            .returns(attitudeSensor)
        justRun { sensorManager.unregisterListener(any(), any<Sensor>()) }

        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, initialLocation)

        val startTimestamp = System.nanoTime()

        val fusedCollector: AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val fusedCollectorSpy = spyk(fusedCollector)
        every { fusedCollectorSpy.start(startTimestamp) }.returns(true)
        estimator.setPrivateProperty("fusedCollector", fusedCollectorSpy)

        val accelerometerFusedCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val accelerometerFusedCollectorSpy = spyk(accelerometerFusedCollector)
        estimator.setPrivateProperty("accelerometerFusedCollector", accelerometerFusedCollectorSpy)

        val attitudeCollector: AttitudeAccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("attitudeCollector")
        requireNotNull(attitudeCollector)
        val attitudeCollectorSpy = spyk(attitudeCollector)
        estimator.setPrivateProperty("attitudeCollector", attitudeCollectorSpy)

        // stop
        estimator.stop()

        verify(exactly = 1) { fusedCollectorSpy.stop() }
        verify(exactly = 1) { accelerometerFusedCollectorSpy.stop() }
        verify(exactly = 1) { attitudeCollectorSpy.stop() }
    }

    @Test
    fun fusedCollector_whenAccuracyChangedAndNoListener_makesNoAction() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, initialLocation)

        val fusedCollector: AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val listener = fusedCollector.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(fusedCollector, SensorType.ABSOLUTE_ATTITUDE, SensorAccuracy.HIGH)
    }

    @Test
    fun fusedCollector_whenAccuracyChangedAndListener_notifies() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            accuracyChangedListener = accuracyChangedListener
        )

        val fusedCollector: AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val listener = fusedCollector.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(fusedCollector, SensorType.ABSOLUTE_ATTITUDE, SensorAccuracy.HIGH)

        // check
        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                estimator,
                SensorType.ABSOLUTE_ATTITUDE,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun fusedCollector_whenSyncedMeasurementUseDoubleFusedAttitudeAndNotProcessed_makesNoAction() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            poseAvailableListener = poseAvailableListener
        )

        val fusedCollector: AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val listener = fusedCollector.measurementListener
        requireNotNull(listener)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        every { doubleFusedProcessorSpy.process(any()) }.returns(false)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        // notify measurement
        val measurement = AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onMeasurement(fusedCollector, measurement)

        // check
        verify(exactly = 1) { doubleFusedProcessorSpy.process(measurement) }
        verify { poseAvailableListener wasNot Called }
    }

    @Test
    fun fusedCollector_whenSyncedMeasurementUseDoubleFusedAttitudeProcessedNotEstimatePoseTransformationAndNoListener_makesExpectedCalls() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = false
        )

        val fusedCollector: AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val listener = fusedCollector.measurementListener
        requireNotNull(listener)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        every { doubleFusedProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        // notify measurement
        val measurement = AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onMeasurement(fusedCollector, measurement)

        // check
        verify(exactly = 1) { doubleFusedProcessorSpy.process(measurement) }
        verify(inverse = true) { doubleFusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { doubleFusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { doubleFusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { doubleFusedProcessorSpy.initialEcefFrame }
    }

    @Test
    fun fusedCollector_whenSyncedMeasurementUseDoubleFusedAttitudeProcessedEstimatePoseTransformationAndNoListener_makesExpectedCalls() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = true
        )

        val fusedCollector: AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val listener = fusedCollector.measurementListener
        requireNotNull(listener)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        every { doubleFusedProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        // notify measurement
        val measurement = AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onMeasurement(fusedCollector, measurement)

        // check
        verify(exactly = 1) { doubleFusedProcessorSpy.process(measurement) }
        verify(exactly = 1) { doubleFusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { doubleFusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { doubleFusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { doubleFusedProcessorSpy.initialEcefFrame }
    }

    @Test
    fun fusedCollector_whenSyncedMeasurementUseDoubleFusedAttitudeProcessedNotEstimatePoseTransformationAndListener_makesExpectedCalls() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = false,
            poseAvailableListener = poseAvailableListener
        )

        val fusedCollector: AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val listener = fusedCollector.measurementListener
        requireNotNull(listener)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        every { doubleFusedProcessorSpy.process(any()) }.returns(true)
        val currentEcefFrame = ECEFFrame()
        every { doubleFusedProcessorSpy.currentEcefFrame }.returns(currentEcefFrame)
        val previousEcefFrame = ECEFFrame()
        every { doubleFusedProcessorSpy.previousEcefFrame }.returns(previousEcefFrame)
        val initialEcefFrame = ECEFFrame()
        every { doubleFusedProcessorSpy.initialEcefFrame }.returns(initialEcefFrame)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        // notify measurement
        val timestamp = System.nanoTime()
        val measurement =
            AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement(timestamp = timestamp)
        listener.onMeasurement(fusedCollector, measurement)

        // check
        verify(exactly = 1) { doubleFusedProcessorSpy.process(measurement) }
        verify(inverse = true) { doubleFusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { doubleFusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { doubleFusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { doubleFusedProcessorSpy.initialEcefFrame }
        verify(exactly = 1) {
            poseAvailableListener.onPoseAvailable(
                estimator,
                currentEcefFrame,
                previousEcefFrame,
                initialEcefFrame,
                timestamp,
                null
            )
        }
    }

    @Test
    fun fusedCollector_whenSyncedMeasurementUseDoubleFusedAttitudeProcessedEstimatePoseTransformationAndListener_makesExpectedCalls() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = true,
            poseAvailableListener = poseAvailableListener
        )

        val fusedCollector: AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val listener = fusedCollector.measurementListener
        requireNotNull(listener)

        val doubleFusedProcessor: DoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        every { doubleFusedProcessorSpy.process(any()) }.returns(true)
        val poseTransformation = EuclideanTransformation3D()
        every { doubleFusedProcessorSpy.poseTransformation }.returns(poseTransformation)
        val currentEcefFrame = ECEFFrame()
        every { doubleFusedProcessorSpy.currentEcefFrame }.returns(currentEcefFrame)
        val previousEcefFrame = ECEFFrame()
        every { doubleFusedProcessorSpy.previousEcefFrame }.returns(previousEcefFrame)
        val initialEcefFrame = ECEFFrame()
        every { doubleFusedProcessorSpy.initialEcefFrame }.returns(initialEcefFrame)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        // notify measurement
        val timestamp = System.nanoTime()
        val measurement =
            AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement(timestamp = timestamp)
        listener.onMeasurement(fusedCollector, measurement)

        // check
        verify(exactly = 1) { doubleFusedProcessorSpy.process(measurement) }
        verify(exactly = 1) { doubleFusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { doubleFusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { doubleFusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { doubleFusedProcessorSpy.initialEcefFrame }
        verify(exactly = 1) {
            poseAvailableListener.onPoseAvailable(
                estimator,
                currentEcefFrame,
                previousEcefFrame,
                initialEcefFrame,
                timestamp,
                poseTransformation
            )
        }
    }

    @Test
    fun fusedCollector_whenSyncedMeasurementNotUseDoubleFusedAttitudeAndNotProcessed_makesNoAction() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = false,
            poseAvailableListener = poseAvailableListener
        )

        val fusedCollector: AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val listener = fusedCollector.measurementListener
        requireNotNull(listener)

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        every { fusedProcessorSpy.process(any()) }.returns(false)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        // notify measurement
        val measurement = AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onMeasurement(fusedCollector, measurement)

        // check
        verify(exactly = 1) { fusedProcessorSpy.process(measurement) }
        verify { poseAvailableListener wasNot Called }
    }

    @Test
    fun fusedCollector_whenSyncedMeasurementNotUseDoubleFusedAttitudeProcessedNotEstimatePoseTransformationAndNoListener_makesExpectedCalls() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = false,
            estimatePoseTransformation = false
        )

        val fusedCollector: AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val listener = fusedCollector.measurementListener
        requireNotNull(listener)

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        every { fusedProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        // notify measurement
        val measurement = AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onMeasurement(fusedCollector, measurement)

        // check
        verify(exactly = 1) { fusedProcessorSpy.process(measurement) }
        verify(inverse = true) { fusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { fusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { fusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { fusedProcessorSpy.initialEcefFrame }
    }

    @Test
    fun fusedCollector_whenSyncedMeasurementNotUseDoubleFusedAttitudeProcessedEstimatePoseTransformationAndNoListener_makesExpectedCalls() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = false,
            estimatePoseTransformation = true
        )

        val fusedCollector: AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val listener = fusedCollector.measurementListener
        requireNotNull(listener)

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        every { fusedProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        // notify measurement
        val measurement = AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onMeasurement(fusedCollector, measurement)

        // check
        verify(exactly = 1) { fusedProcessorSpy.process(measurement) }
        verify(exactly = 1) { fusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { fusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { fusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { fusedProcessorSpy.initialEcefFrame }
    }

    @Test
    fun fusedCollector_whenSyncedMeasurementNotUseDoubleFusedAttitudeProcessedNotEstimatePoseTransformationAndListener_makesExpectedCalls() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = false,
            estimatePoseTransformation = false,
            poseAvailableListener = poseAvailableListener
        )

        val fusedCollector: AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val listener = fusedCollector.measurementListener
        requireNotNull(listener)

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        every { fusedProcessorSpy.process(any()) }.returns(true)
        val currentEcefFrame = ECEFFrame()
        every { fusedProcessorSpy.currentEcefFrame }.returns(currentEcefFrame)
        val previousEcefFrame = ECEFFrame()
        every { fusedProcessorSpy.previousEcefFrame }.returns(previousEcefFrame)
        val initialEcefFrame = ECEFFrame()
        every { fusedProcessorSpy.initialEcefFrame }.returns(initialEcefFrame)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        // notify measurement
        val timestamp = System.nanoTime()
        val measurement =
            AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement(timestamp = timestamp)
        listener.onMeasurement(fusedCollector, measurement)

        // check
        verify(exactly = 1) { fusedProcessorSpy.process(measurement) }
        verify(inverse = true) { fusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { fusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { fusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { fusedProcessorSpy.initialEcefFrame }
        verify(exactly = 1) {
            poseAvailableListener.onPoseAvailable(
                estimator,
                currentEcefFrame,
                previousEcefFrame,
                initialEcefFrame,
                timestamp,
                null
            )
        }
    }

    @Test
    fun fusedCollector_whenSyncedMeasurementNotUseDoubleFusedAttitudeProcessedEstimatePoseTransformationAndListener_makesExpectedCalls() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = false,
            estimatePoseTransformation = true,
            poseAvailableListener = poseAvailableListener
        )

        val fusedCollector: AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val listener = fusedCollector.measurementListener
        requireNotNull(listener)

        val fusedProcessor: FusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        every { fusedProcessorSpy.process(any()) }.returns(true)
        val poseTransformation = EuclideanTransformation3D()
        every { fusedProcessorSpy.poseTransformation }.returns(poseTransformation)
        val currentEcefFrame = ECEFFrame()
        every { fusedProcessorSpy.currentEcefFrame }.returns(currentEcefFrame)
        val previousEcefFrame = ECEFFrame()
        every { fusedProcessorSpy.previousEcefFrame }.returns(previousEcefFrame)
        val initialEcefFrame = ECEFFrame()
        every { fusedProcessorSpy.initialEcefFrame }.returns(initialEcefFrame)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        // notify measurement
        val timestamp = System.nanoTime()
        val measurement =
            AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement(timestamp = timestamp)
        listener.onMeasurement(fusedCollector, measurement)

        // check
        verify(exactly = 1) { fusedProcessorSpy.process(measurement) }
        verify(exactly = 1) { fusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { fusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { fusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { fusedProcessorSpy.initialEcefFrame }
        verify(exactly = 1) {
            poseAvailableListener.onPoseAvailable(
                estimator,
                currentEcefFrame,
                previousEcefFrame,
                initialEcefFrame,
                timestamp,
                poseTransformation
            )
        }
    }

    @Test
    fun accelerometerFusedCollector_whenAccuracyChangedAndNoListener_makesNoAction() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, initialLocation)

        val accelerometerFusedCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val listener = accelerometerFusedCollector.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(
            accelerometerFusedCollector,
            SensorType.ABSOLUTE_ATTITUDE,
            SensorAccuracy.HIGH
        )
    }

    @Test
    fun accelerometerFusedCollector_whenAccuracyChangedAndListener_notifies() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            accuracyChangedListener = accuracyChangedListener
        )

        val accelerometerFusedCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val listener = accelerometerFusedCollector.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(
            accelerometerFusedCollector,
            SensorType.ABSOLUTE_ATTITUDE,
            SensorAccuracy.HIGH
        )

        // check
        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                estimator,
                SensorType.ABSOLUTE_ATTITUDE,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun accelerometerFusedCollector_whenSyncedMeasurementUseDoubleFusedAttitudeAndNotProcessed_makesNoAction() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            poseAvailableListener = poseAvailableListener
        )

        val accelerometerFusedCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val listener = accelerometerFusedCollector.measurementListener
        requireNotNull(listener)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        every { accelerometerDoubleFusedProcessorSpy.process(any()) }.returns(false)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        // notify measurement
        val measurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onMeasurement(accelerometerFusedCollector, measurement)

        // check
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.process(measurement) }
        verify { poseAvailableListener wasNot Called }
    }

    @Test
    fun accelerometerFusedSyncer_whenSyncedMeasurementUseDoubleFusedAttitudeProcessedNotEstimatePoseTransformationAndNoListener_makesExpectedCalls() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = false
        )

        val accelerometerFusedCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val listener = accelerometerFusedCollector.measurementListener
        requireNotNull(listener)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        every { accelerometerDoubleFusedProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        // notify measurement
        val measurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onMeasurement(accelerometerFusedCollector, measurement)

        // check
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.process(measurement) }
        verify(inverse = true) { accelerometerDoubleFusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.initialEcefFrame }
    }

    @Test
    fun accelerometerFusedSyncer_whenSyncedMeasurementUseDoubleFusedAttitudeProcessedEstimatePoseTransformationAndNoListener_makesExpectedCalls() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = true
        )

        val accelerometerFusedCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val listener = accelerometerFusedCollector.measurementListener
        requireNotNull(listener)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        every { accelerometerDoubleFusedProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        // notify measurement
        val measurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onMeasurement(accelerometerFusedCollector, measurement)

        // check
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.process(measurement) }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.initialEcefFrame }
    }

    @Test
    fun accelerometerFusedCollector_whenSyncedMeasurementUseDoubleFusedAttitudeProcessedNotEstimatePoseTransformationAndListener_makesExpectedCalls() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = false,
            poseAvailableListener = poseAvailableListener
        )

        val accelerometerFusedCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val listener = accelerometerFusedCollector.measurementListener
        requireNotNull(listener)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        every { accelerometerDoubleFusedProcessorSpy.process(any()) }.returns(true)
        val currentEcefFrame = ECEFFrame()
        every { accelerometerDoubleFusedProcessorSpy.currentEcefFrame }.returns(currentEcefFrame)
        val previousEcefFrame = ECEFFrame()
        every { accelerometerDoubleFusedProcessorSpy.previousEcefFrame }.returns(previousEcefFrame)
        val initialEcefFrame = ECEFFrame()
        every { accelerometerDoubleFusedProcessorSpy.initialEcefFrame }.returns(initialEcefFrame)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        // notify measurement
        val timestamp = System.nanoTime()
        val measurement =
            AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement(timestamp = timestamp)
        listener.onMeasurement(accelerometerFusedCollector, measurement)

        // check
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.process(measurement) }
        verify(inverse = true) { accelerometerDoubleFusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.initialEcefFrame }
        verify(exactly = 1) {
            poseAvailableListener.onPoseAvailable(
                estimator,
                currentEcefFrame,
                previousEcefFrame,
                initialEcefFrame,
                timestamp,
                null
            )
        }
    }

    @Test
    fun accelerometerFusedCollector_whenSyncedMeasurementUseDoubleFusedAttitudeProcessedEstimatePoseTransformationAndListener_makesExpectedCalls() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = true,
            poseAvailableListener = poseAvailableListener
        )

        val accelerometerFusedCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val listener = accelerometerFusedCollector.measurementListener
        requireNotNull(listener)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        every { accelerometerDoubleFusedProcessorSpy.process(any()) }.returns(true)
        val poseTransformation = EuclideanTransformation3D()
        every { accelerometerDoubleFusedProcessorSpy.poseTransformation }.returns(poseTransformation)
        val currentEcefFrame = ECEFFrame()
        every { accelerometerDoubleFusedProcessorSpy.currentEcefFrame }.returns(currentEcefFrame)
        val previousEcefFrame = ECEFFrame()
        every { accelerometerDoubleFusedProcessorSpy.previousEcefFrame }.returns(previousEcefFrame)
        val initialEcefFrame = ECEFFrame()
        every { accelerometerDoubleFusedProcessorSpy.initialEcefFrame }.returns(initialEcefFrame)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        // notify measurement
        val timestamp = System.nanoTime()
        val measurement =
            AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement(timestamp = timestamp)
        listener.onMeasurement(accelerometerFusedCollector, measurement)

        // check
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.process(measurement) }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.initialEcefFrame }
        verify(exactly = 1) {
            poseAvailableListener.onPoseAvailable(
                estimator,
                currentEcefFrame,
                previousEcefFrame,
                initialEcefFrame,
                timestamp,
                poseTransformation
            )
        }
    }

    @Test
    fun accelerometerFusedCollector_whenSyncedMeasurementNotUseDoubleFusedAttitudeAndNotProcessed_makesNoAction() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = false,
            poseAvailableListener = poseAvailableListener
        )

        val accelerometerFusedCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val listener = accelerometerFusedCollector.measurementListener
        requireNotNull(listener)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        every { accelerometerFusedProcessorSpy.process(any()) }.returns(false)
        estimator.setPrivateProperty(
            "accelerometerFusedProcessor",
            accelerometerFusedProcessorSpy
        )

        // notify measurement
        val measurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onMeasurement(accelerometerFusedCollector, measurement)

        // check
        verify(exactly = 1) { accelerometerFusedProcessorSpy.process(measurement) }
        verify { poseAvailableListener wasNot Called }
    }

    @Test
    fun accelerometerFusedCollector_whenSyncedMeasurementNotUseDoubleFusedAttitudeProcessedNotEstimatePoseTransformationAndNoListener_makesExpectedCalls() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = false,
            estimatePoseTransformation = false
        )

        val accelerometerFusedCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val listener = accelerometerFusedCollector.measurementListener
        requireNotNull(listener)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        every { accelerometerFusedProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerFusedProcessor",
            accelerometerFusedProcessorSpy
        )

        // notify measurement
        val measurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onMeasurement(accelerometerFusedCollector, measurement)

        // check
        verify(exactly = 1) { accelerometerFusedProcessorSpy.process(measurement) }
        verify(inverse = true) { accelerometerFusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.initialEcefFrame }
    }

    @Test
    fun accelerometerFusedCollector_whenSyncedMeasurementNotUseDoubleFusedAttitudeProcessedEstimatePoseTransformationAndNoListener_makesExpectedCalls() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = false,
            estimatePoseTransformation = true
        )

        val accelerometerFusedCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val listener = accelerometerFusedCollector.measurementListener
        requireNotNull(listener)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        every { accelerometerFusedProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        // notify measurement
        val measurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onMeasurement(accelerometerFusedCollector, measurement)

        // check
        verify(exactly = 1) { accelerometerFusedProcessorSpy.process(measurement) }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.initialEcefFrame }
    }

    @Test
    fun accelerometerFusedCollector_whenSyncedMeasurementNotUseDoubleFusedAttitudeProcessedNotEstimatePoseTransformationAndListener_makesExpectedCalls() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = false,
            estimatePoseTransformation = false,
            poseAvailableListener = poseAvailableListener
        )

        val accelerometerFusedCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val listener = accelerometerFusedCollector.measurementListener
        requireNotNull(listener)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        every { accelerometerFusedProcessorSpy.process(any()) }.returns(true)
        val currentEcefFrame = ECEFFrame()
        every { accelerometerFusedProcessorSpy.currentEcefFrame }.returns(currentEcefFrame)
        val previousEcefFrame = ECEFFrame()
        every { accelerometerFusedProcessorSpy.previousEcefFrame }.returns(previousEcefFrame)
        val initialEcefFrame = ECEFFrame()
        every { accelerometerFusedProcessorSpy.initialEcefFrame }.returns(initialEcefFrame)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        // notify measurement
        val timestamp = System.nanoTime()
        val measurement =
            AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement(timestamp = timestamp)
        listener.onMeasurement(accelerometerFusedCollector, measurement)

        // check
        verify(exactly = 1) { accelerometerFusedProcessorSpy.process(measurement) }
        verify(inverse = true) { accelerometerFusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.initialEcefFrame }
        verify(exactly = 1) {
            poseAvailableListener.onPoseAvailable(
                estimator,
                currentEcefFrame,
                previousEcefFrame,
                initialEcefFrame,
                timestamp,
                null
            )
        }
    }

    @Test
    fun accelerometerFusedCollector_whenSyncedMeasurementNotUseDoubleFusedAttitudeProcessedEstimatePoseTransformationAndListener_makesExpectedCalls() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = false,
            estimatePoseTransformation = true,
            poseAvailableListener = poseAvailableListener
        )

        val accelerometerFusedCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val listener = accelerometerFusedCollector.measurementListener
        requireNotNull(listener)

        val accelerometerFusedProcessor: AccelerometerFusedECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        every { accelerometerFusedProcessorSpy.process(any()) }.returns(true)
        val poseTransformation = EuclideanTransformation3D()
        every { accelerometerFusedProcessorSpy.poseTransformation }.returns(poseTransformation)
        val currentEcefFrame = ECEFFrame()
        every { accelerometerFusedProcessorSpy.currentEcefFrame }.returns(currentEcefFrame)
        val previousEcefFrame = ECEFFrame()
        every { accelerometerFusedProcessorSpy.previousEcefFrame }.returns(previousEcefFrame)
        val initialEcefFrame = ECEFFrame()
        every { accelerometerFusedProcessorSpy.initialEcefFrame }.returns(initialEcefFrame)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        // notify measurement
        val timestamp = System.nanoTime()
        val measurement =
            AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement(timestamp = timestamp)
        listener.onMeasurement(accelerometerFusedCollector, measurement)

        // check
        verify(exactly = 1) { accelerometerFusedProcessorSpy.process(measurement) }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.initialEcefFrame }
        verify(exactly = 1) {
            poseAvailableListener.onPoseAvailable(
                estimator,
                currentEcefFrame,
                previousEcefFrame,
                initialEcefFrame,
                timestamp,
                poseTransformation
            )
        }
    }

    @Test
    fun attitudeCollector_whenAccuracyChangedAndNoListener_makesNoAction() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, initialLocation)

        val attitudeCollector: AttitudeAccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("attitudeCollector")
        requireNotNull(attitudeCollector)
        val listener = attitudeCollector.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(
            attitudeCollector,
            SensorType.ABSOLUTE_ATTITUDE,
            SensorAccuracy.HIGH
        )
    }

    @Test
    fun attitudeCollector_whenAccuracyChangedAndListener_notifies() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            accuracyChangedListener = accuracyChangedListener
        )

        val attitudeCollector: AttitudeAccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("attitudeCollector")
        requireNotNull(attitudeCollector)
        val listener = attitudeCollector.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(
            attitudeCollector,
            SensorType.ABSOLUTE_ATTITUDE,
            SensorAccuracy.HIGH
        )

        // check
        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                estimator,
                SensorType.ABSOLUTE_ATTITUDE,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun attitudeCollector_whenSyncedMeasurementAndNotProcessed_makesNoAction() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            poseAvailableListener = poseAvailableListener
        )

        val attitudeCollector: AttitudeAccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("attitudeCollector")
        requireNotNull(attitudeCollector)
        val listener = attitudeCollector.measurementListener
        requireNotNull(listener)

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        every { attitudeProcessorSpy.process(any()) }.returns(false)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // notify measurement
        val measurement = AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement()
        listener.onMeasurement(attitudeCollector, measurement)

        // check
        verify(exactly = 1) { attitudeProcessorSpy.process(measurement) }
        verify { poseAvailableListener wasNot Called }
    }

    @Test
    fun attitudeCollector_whenSyncedMeasurementProcessedNotEstimatePoseTransformationAndNoListener_makesExpectedCalls() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = false
        )

        val attitudeCollector: AttitudeAccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("attitudeCollector")
        requireNotNull(attitudeCollector)
        val listener = attitudeCollector.measurementListener
        requireNotNull(listener)

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        every { attitudeProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // notify measurement
        val measurement = AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement()
        listener.onMeasurement(attitudeCollector, measurement)

        // check
        verify(exactly = 1) { attitudeProcessorSpy.process(measurement) }
        verify(inverse = true) { attitudeProcessorSpy.poseTransformation }
        verify(exactly = 1) { attitudeProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { attitudeProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { attitudeProcessorSpy.initialEcefFrame }
    }

    @Test
    fun attitudeCollector_whenSyncedMeasurementAttitudeProcessedEstimatePoseTransformationAndNoListener_makesExpectedCalls() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = true
        )

        val attitudeCollector: AttitudeAccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("attitudeCollector")
        requireNotNull(attitudeCollector)
        val listener = attitudeCollector.measurementListener
        requireNotNull(listener)

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        every { attitudeProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // notify measurement
        val measurement = AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement()
        listener.onMeasurement(attitudeCollector, measurement)

        // check
        verify(exactly = 1) { attitudeProcessorSpy.process(measurement) }
        verify(exactly = 1) { attitudeProcessorSpy.poseTransformation }
        verify(exactly = 1) { attitudeProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { attitudeProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { attitudeProcessorSpy.initialEcefFrame }
    }

    @Test
    fun attitudeCollector_whenSyncedMeasurementAttitudeProcessedNotEstimatePoseTransformationAndListener_makesExpectedCalls() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = false,
            poseAvailableListener = poseAvailableListener
        )

        val attitudeCollector: AttitudeAccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("attitudeCollector")
        requireNotNull(attitudeCollector)
        val listener = attitudeCollector.measurementListener
        requireNotNull(listener)

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        every { attitudeProcessorSpy.process(any()) }.returns(true)
        val currentEcefFrame = ECEFFrame()
        every { attitudeProcessorSpy.currentEcefFrame }.returns(currentEcefFrame)
        val previousEcefFrame = ECEFFrame()
        every { attitudeProcessorSpy.previousEcefFrame }.returns(previousEcefFrame)
        val initialEcefFrame = ECEFFrame()
        every { attitudeProcessorSpy.initialEcefFrame }.returns(initialEcefFrame)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // notify measurement
        val timestamp = System.nanoTime()
        val measurement =
            AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement(timestamp = timestamp)
        listener.onMeasurement(attitudeCollector, measurement)

        // check
        verify(exactly = 1) { attitudeProcessorSpy.process(measurement) }
        verify(inverse = true) { attitudeProcessorSpy.poseTransformation }
        verify(exactly = 1) { attitudeProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { attitudeProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { attitudeProcessorSpy.initialEcefFrame }
        verify(exactly = 1) {
            poseAvailableListener.onPoseAvailable(
                estimator,
                currentEcefFrame,
                previousEcefFrame,
                initialEcefFrame,
                timestamp,
                null
            )
        }
    }

    @Test
    fun attitudeSyncer_whenSyncedMeasurementUseDoubleFusedAttitudeProcessedEstimatePoseTransformationAndListener_makesExpectedCalls() {
        val initialLocation = getLocation()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = true,
            poseAvailableListener = poseAvailableListener
        )

        val attitudeCollector: AttitudeAccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("attitudeCollector")
        requireNotNull(attitudeCollector)
        val listener = attitudeCollector.measurementListener
        requireNotNull(listener)

        val attitudeProcessor: AttitudeECEFAbsolutePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        every { attitudeProcessorSpy.process(any()) }.returns(true)
        val poseTransformation = EuclideanTransformation3D()
        every { attitudeProcessorSpy.poseTransformation }.returns(poseTransformation)
        val currentEcefFrame = ECEFFrame()
        every { attitudeProcessorSpy.currentEcefFrame }.returns(currentEcefFrame)
        val previousEcefFrame = ECEFFrame()
        every { attitudeProcessorSpy.previousEcefFrame }.returns(previousEcefFrame)
        val initialEcefFrame = ECEFFrame()
        every { attitudeProcessorSpy.initialEcefFrame }.returns(initialEcefFrame)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // notify measurement
        val timestamp = System.nanoTime()
        val measurement =
            AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement(timestamp = timestamp)
        listener.onMeasurement(attitudeCollector, measurement)

        // check
        verify(exactly = 1) { attitudeProcessorSpy.process(measurement) }
        verify(exactly = 1) { attitudeProcessorSpy.poseTransformation }
        verify(exactly = 1) { attitudeProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { attitudeProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { attitudeProcessorSpy.initialEcefFrame }
        verify(exactly = 1) {
            poseAvailableListener.onPoseAvailable(
                estimator,
                currentEcefFrame,
                previousEcefFrame,
                initialEcefFrame,
                timestamp,
                poseTransformation
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

        const val TIME_INTERVAL = 0.02
    }
}