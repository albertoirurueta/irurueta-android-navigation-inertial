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
package com.irurueta.android.navigation.inertial.old.estimators.pose

import android.content.Context
import android.location.Location
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.navigation.inertial.old.collectors.AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer
import com.irurueta.android.navigation.inertial.old.collectors.AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.old.collectors.AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer
import com.irurueta.android.navigation.inertial.old.collectors.AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.old.collectors.AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer
import com.irurueta.android.navigation.inertial.old.collectors.AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.old.collectors.SensorType
import com.irurueta.android.navigation.inertial.old.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.old.estimators.filter.MedianAveragingFilter
import com.irurueta.android.navigation.inertial.old.processors.attitude.BaseDoubleFusedGeomagneticAttitudeProcessor
import com.irurueta.android.navigation.inertial.old.processors.attitude.BaseFusedGeomagneticAttitudeProcessor
import com.irurueta.android.navigation.inertial.old.processors.pose.AccelerometerDoubleFusedLocalPoseProcessor
import com.irurueta.android.navigation.inertial.old.processors.pose.AccelerometerFusedLocalPoseProcessor
import com.irurueta.android.navigation.inertial.old.processors.pose.AttitudeLocalPoseProcessor
import com.irurueta.android.navigation.inertial.old.processors.pose.DoubleFusedLocalPoseProcessor
import com.irurueta.android.navigation.inertial.old.processors.pose.FusedLocalPoseProcessor
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.geometry.EuclideanTransformation3D
import com.irurueta.navigation.frames.ECEFFrame
import com.irurueta.navigation.frames.NEDVelocity
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.statistics.UniformRandomizer
import io.mockk.Called
import io.mockk.every
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import io.mockk.spyk
import io.mockk.verify
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
import java.util.Date

@RunWith(RobolectricTestRunner::class)
class LocalPoseEstimator2Test {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var poseAvailableListener: LocalPoseEstimator2.OnPoseAvailableListener

    @MockK(relaxUnitFun = true)
    private lateinit var accuracyChangedListener: LocalPoseEstimator2.OnAccuracyChangedListener

    @MockK(relaxUnitFun = true)
    private lateinit var bufferFilledListener: LocalPoseEstimator2.OnBufferFilledListener

    @MockK
    private lateinit var location: Location

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(context, initialLocation)

        // check
        assertSame(context, estimator.context)
        assertSame(initialLocation, estimator.initialLocation)
        assertEquals(NEDVelocity(), estimator.initialVelocity)
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertTrue(estimator.useAttitudeSensor)
        assertFalse(estimator.useAccelerometerForAttitudeEstimation)
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
        assertNull(estimator.bufferFilledListener)
        assertTrue(estimator.adjustGravityNorm)
    }

    @Test
    fun constructor_whenAllProperties_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val initialVelocity = NEDVelocity()
        val accelerometerAveragingFilter = MedianAveragingFilter()
        val worldMagneticModel = WorldMagneticModel()
        val timestamp = Date()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            initialVelocity,
            SensorDelay.NORMAL,
            useAttitudeSensor = false,
            useAccelerometerForAttitudeEstimation = true,
            startOffsetEnabled = false,
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
            bufferFilledListener
        )

        // check
        assertSame(context, estimator.context)
        assertSame(initialLocation, estimator.initialLocation)
        assertSame(initialVelocity, estimator.initialVelocity)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertFalse(estimator.useAttitudeSensor)
        assertTrue(estimator.useAccelerometerForAttitudeEstimation)
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
        assertSame(bufferFilledListener, estimator.bufferFilledListener)
        assertTrue(estimator.adjustGravityNorm)
    }

    @Test
    fun initialLocation_whenUseAttitudeSensor_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator =
            LocalPoseEstimator2(context, initialLocation, useAttitudeSensor = true)

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator =
            LocalPoseEstimator2(
                context,
                initialLocation,
                useAttitudeSensor = false,
                useDoubleFusedAttitudeProcessor = true,
                useAccelerometerForAttitudeEstimation = true
            )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator =
            LocalPoseEstimator2(
                context,
                initialLocation,
                useAttitudeSensor = false,
                useDoubleFusedAttitudeProcessor = true,
                useAccelerometerForAttitudeEstimation = false
            )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator =
            LocalPoseEstimator2(
                context,
                initialLocation,
                useAttitudeSensor = false,
                useDoubleFusedAttitudeProcessor = false,
                useAccelerometerForAttitudeEstimation = true
            )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator =
            LocalPoseEstimator2(
                context,
                initialLocation,
                useAttitudeSensor = false,
                useDoubleFusedAttitudeProcessor = false,
                useAccelerometerForAttitudeEstimation = false
            )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val initialVelocity = NEDVelocity()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            initialVelocity,
            useAttitudeSensor = true
        )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val initialVelocity = NEDVelocity()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            initialVelocity,
            useAttitudeSensor = false,
            useDoubleFusedAttitudeProcessor = true,
            useAccelerometerForAttitudeEstimation = true
        )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val initialVelocity = NEDVelocity()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            initialVelocity,
            useAttitudeSensor = false,
            useDoubleFusedAttitudeProcessor = true,
            useAccelerometerForAttitudeEstimation = false
        )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val initialVelocity = NEDVelocity()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            initialVelocity,
            useAttitudeSensor = false,
            useDoubleFusedAttitudeProcessor = false,
            useAccelerometerForAttitudeEstimation = true
        )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val initialVelocity = NEDVelocity()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            initialVelocity,
            useAttitudeSensor = false,
            useDoubleFusedAttitudeProcessor = false,
            useAccelerometerForAttitudeEstimation = false
        )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useAttitudeSensor = true
        )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useAttitudeSensor = false,
            useDoubleFusedAttitudeProcessor = true,
            useAccelerometerForAttitudeEstimation = true
        )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useAttitudeSensor = false,
            useDoubleFusedAttitudeProcessor = true,
            useAccelerometerForAttitudeEstimation = false
        )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useAttitudeSensor = false,
            useDoubleFusedAttitudeProcessor = false,
            useAccelerometerForAttitudeEstimation = true
        )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useAttitudeSensor = false,
            useDoubleFusedAttitudeProcessor = false,
            useAccelerometerForAttitudeEstimation = false
        )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useAccurateLevelingProcessor = true
        )

        // check default value
        assertTrue(estimator.useAccurateLevelingProcessor)

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        assertTrue(fusedProcessor.useAccurateLevelingProcessor)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        assertTrue(accelerometerFusedProcessor.useAccurateLevelingProcessor)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        assertTrue(doubleFusedProcessor.useAccurateLevelingProcessor)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        assertTrue(accelerometerDoubleFusedProcessor.useAccurateLevelingProcessor)
    }

    @Test
    fun worldMagneticModel_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val worldMagneticModel = WorldMagneticModel()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            worldMagneticModel = worldMagneticModel
        )

        // check default value
        assertSame(worldMagneticModel, estimator.worldMagneticModel)

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        assertSame(worldMagneticModel, fusedProcessor.worldMagneticModel)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        assertSame(worldMagneticModel, accelerometerFusedProcessor.worldMagneticModel)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        assertSame(worldMagneticModel, doubleFusedProcessor.worldMagneticModel)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        assertSame(worldMagneticModel, accelerometerDoubleFusedProcessor.worldMagneticModel)
    }

    @Test
    fun useWorldMagneticModel_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useWorldMagneticModel = true
        )

        // check default value
        assertTrue(estimator.useWorldMagneticModel)

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        assertTrue(fusedProcessor.useWorldMagneticModel)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        assertTrue(accelerometerFusedProcessor.useWorldMagneticModel)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        assertTrue(doubleFusedProcessor.useWorldMagneticModel)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        assertTrue(accelerometerDoubleFusedProcessor.useWorldMagneticModel)
    }

    @Test
    fun timestamp_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation
        )

        // check default value
        assertNull(estimator.timestamp)

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation
        )

        // check default value
        assertTrue(estimator.useAccurateRelativeGyroscopeAttitudeProcessor)

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        assertTrue(fusedProcessor.useAccurateRelativeGyroscopeAttitudeProcessor)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        assertTrue(accelerometerFusedProcessor.useAccurateRelativeGyroscopeAttitudeProcessor)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        assertTrue(doubleFusedProcessor.useAccurateRelativeGyroscopeAttitudeProcessor)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        assertTrue(accelerometerDoubleFusedProcessor.useAccurateRelativeGyroscopeAttitudeProcessor)
    }

    @Test
    fun useIndirectAttitudeInterpolation_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation
        )

        // check default value
        assertTrue(estimator.useIndirectAttitudeInterpolation)

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation
        )

        // check default value
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            estimator.attitudeInterpolationValue,
            0.0
        )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation
        )

        // check default value
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.attitudeIndirectInterpolationWeight,
            0.0
        )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation
        )

        // check default value
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            estimator.attitudeOutlierThreshold,
            0.0
        )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation
        )

        // check default value
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.attitudeOutlierPanicThreshold,
            0.0
        )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation
        )

        // check default value
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.attitudePanicCounterThreshold,
        )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator =
            LocalPoseEstimator2(context, initialLocation, useAttitudeSensor = true)

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator =
            LocalPoseEstimator2(
                context,
                initialLocation,
                useAttitudeSensor = false,
                useDoubleFusedAttitudeProcessor = true,
                useAccelerometerForAttitudeEstimation = true
            )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
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

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator =
            LocalPoseEstimator2(
                context,
                initialLocation,
                useAttitudeSensor = false,
                useDoubleFusedAttitudeProcessor = true,
                useAccelerometerForAttitudeEstimation = false
            )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        every { doubleFusedProcessorSpy.gyroscopeTimeIntervalSeconds }.returns(
            TIME_INTERVAL
        )
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator =
            LocalPoseEstimator2(
                context,
                initialLocation,
                useAttitudeSensor = false,
                useDoubleFusedAttitudeProcessor = false,
                useAccelerometerForAttitudeEstimation = true
            )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        every { accelerometerFusedProcessorSpy.gyroscopeTimeIntervalSeconds }.returns(
            TIME_INTERVAL
        )
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val timeIntervalSeconds = estimator.timeIntervalSeconds
        requireNotNull(timeIntervalSeconds)
        assertEquals(TIME_INTERVAL, timeIntervalSeconds, 0.0)

        verify { fusedProcessorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.gyroscopeTimeIntervalSeconds }
    }

    @Test
    fun timeIntervalSeconds_whenNotUseAttitudeSensorNotUseDoubleFusedAttitudeProcessorAndNotUseAccelerometerForAttitudeEstimation_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator =
            LocalPoseEstimator2(
                context,
                initialLocation,
                useAttitudeSensor = false,
                useDoubleFusedAttitudeProcessor = false,
                useAccelerometerForAttitudeEstimation = false
            )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        every { fusedProcessorSpy.gyroscopeTimeIntervalSeconds }.returns(
            TIME_INTERVAL
        )
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val timeIntervalSeconds = estimator.timeIntervalSeconds
        requireNotNull(timeIntervalSeconds)
        assertEquals(TIME_INTERVAL, timeIntervalSeconds, 0.0)

        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
        verify(exactly = 1) { fusedProcessorSpy.gyroscopeTimeIntervalSeconds }
    }

    @Test
    fun useLeveledRelativeAttitudeRespectStart_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation
        )

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(context, initialLocation)

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
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

    @Test(expected = IllegalStateException::class)
    fun adjustGravityNorm_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(context, initialLocation)

        assertFalse(estimator.running)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        estimator.adjustGravityNorm = false
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation
        )

        assertFalse(estimator.running)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        estimator.start()

        assertFalse(estimator.running)
    }

    @Test
    fun start_whenUseAttitudeSensor_resetsProcessorAndStartsSyncer() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useAttitudeSensor = true
        )

        val startTimestamp = System.nanoTime()

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val fusedSyncer: AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val fusedSyncerSpy = spyk(fusedSyncer)
        estimator.setPrivateProperty("fusedSyncer", fusedSyncerSpy)

        val accelerometerFusedSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val accelerometerFusedSyncerSpy = spyk(accelerometerFusedSyncer)
        estimator.setPrivateProperty("accelerometerFusedSyncer", accelerometerFusedSyncerSpy)

        val attitudeSyncer: AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val attitudeSyncerSpy = spyk(attitudeSyncer)
        every { attitudeSyncerSpy.start(startTimestamp) }.returns(true)
        estimator.setPrivateProperty("attitudeSyncer", attitudeSyncerSpy)

        assertTrue(estimator.start(startTimestamp))
        assertTrue(estimator.running)

        verify { attitudeProcessorSpy.reset() }
        verify(exactly = 1) { attitudeSyncerSpy.start(startTimestamp) }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { accelerometerFusedSyncerSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { fusedProcessorSpy wasNot Called }
        verify { fusedSyncerSpy wasNot Called }
    }

    @Test
    fun start_whenNotUseAttitudeSensorUseAccelerometerForAttitudeEstimationAndUseDoubleFusedAttitudeProcessor_resetsProcessorAndStartsSyncer() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useAttitudeSensor = false,
            useAccelerometerForAttitudeEstimation = true,
            useDoubleFusedAttitudeProcessor = true
        )

        val startTimestamp = System.nanoTime()

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val fusedSyncer: AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val fusedSyncerSpy = spyk(fusedSyncer)
        estimator.setPrivateProperty("fusedSyncer", fusedSyncerSpy)

        val accelerometerFusedSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val accelerometerFusedSyncerSpy = spyk(accelerometerFusedSyncer)
        every { accelerometerFusedSyncerSpy.start(startTimestamp) }.returns(true)
        estimator.setPrivateProperty("accelerometerFusedSyncer", accelerometerFusedSyncerSpy)

        val attitudeSyncer: AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val attitudeSyncerSpy = spyk(attitudeSyncer)
        estimator.setPrivateProperty("attitudeSyncer", attitudeSyncerSpy)

        assertTrue(estimator.start(startTimestamp))
        assertTrue(estimator.running)

        verify { accelerometerDoubleFusedProcessorSpy.reset() }
        verify(exactly = 1) { accelerometerFusedSyncerSpy.start(startTimestamp) }
        verify { attitudeProcessorSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { attitudeSyncerSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { fusedProcessorSpy wasNot Called }
        verify { fusedSyncerSpy wasNot Called }
    }

    @Test
    fun start_whenNotUseAttitudeSensorUseAccelerometerForAttitudeEstimationAndNotUseDoubleFusedAttitudeProcessor_resetsProcessorAndStartsSyncer() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useAttitudeSensor = false,
            useAccelerometerForAttitudeEstimation = true,
            useDoubleFusedAttitudeProcessor = false
        )

        val startTimestamp = System.nanoTime()

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val fusedSyncer: AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val fusedSyncerSpy = spyk(fusedSyncer)
        estimator.setPrivateProperty("fusedSyncer", fusedSyncerSpy)

        val accelerometerFusedSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val accelerometerFusedSyncerSpy = spyk(accelerometerFusedSyncer)
        every { accelerometerFusedSyncerSpy.start(startTimestamp) }.returns(true)
        estimator.setPrivateProperty("accelerometerFusedSyncer", accelerometerFusedSyncerSpy)

        val attitudeSyncer: AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val attitudeSyncerSpy = spyk(attitudeSyncer)
        estimator.setPrivateProperty("attitudeSyncer", attitudeSyncerSpy)

        assertTrue(estimator.start(startTimestamp))
        assertTrue(estimator.running)

        verify { accelerometerFusedProcessorSpy.reset() }
        verify(exactly = 1) { accelerometerFusedSyncerSpy.start(startTimestamp) }
        verify { attitudeProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
        verify { attitudeSyncerSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { fusedProcessorSpy wasNot Called }
        verify { fusedSyncerSpy wasNot Called }
    }

    @Test
    fun start_whenNotUseAttitudeSensorNotUseAccelerometerForAttitudeEstimationAndUseDoubleFusedAttitudeProcessor_resetsProcessorAndStartsSyncer() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useAttitudeSensor = false,
            useAccelerometerForAttitudeEstimation = false,
            useDoubleFusedAttitudeProcessor = true
        )

        val startTimestamp = System.nanoTime()

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val fusedSyncer: AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val fusedSyncerSpy = spyk(fusedSyncer)
        every { fusedSyncerSpy.start(startTimestamp) }.returns(true)
        estimator.setPrivateProperty("fusedSyncer", fusedSyncerSpy)

        val accelerometerFusedSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val accelerometerFusedSyncerSpy = spyk(accelerometerFusedSyncer)
        estimator.setPrivateProperty("accelerometerFusedSyncer", accelerometerFusedSyncerSpy)

        val attitudeSyncer: AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val attitudeSyncerSpy = spyk(attitudeSyncer)
        estimator.setPrivateProperty("attitudeSyncer", attitudeSyncerSpy)

        assertTrue(estimator.start(startTimestamp))
        assertTrue(estimator.running)

        verify { doubleFusedProcessorSpy.reset() }
        verify(exactly = 1) { fusedSyncerSpy.start(startTimestamp) }
        verify { attitudeProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
        verify { attitudeSyncerSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { fusedProcessorSpy wasNot Called }
        verify { accelerometerFusedSyncerSpy wasNot Called }
    }

    @Test
    fun start_whenNotUseAttitudeSensorNotUseAccelerometerForAttitudeEstimationAndNotUseDoubleFusedAttitudeProcessor_resetsProcessorAndStartsSyncer() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useAttitudeSensor = false,
            useAccelerometerForAttitudeEstimation = false,
            useDoubleFusedAttitudeProcessor = false
        )

        val startTimestamp = System.nanoTime()

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerDoubleFusedProcessor")
        requireNotNull(accelerometerDoubleFusedProcessor)
        val accelerometerDoubleFusedProcessorSpy = spyk(accelerometerDoubleFusedProcessor)
        estimator.setPrivateProperty(
            "accelerometerDoubleFusedProcessor",
            accelerometerDoubleFusedProcessorSpy
        )

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val fusedSyncer: AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val fusedSyncerSpy = spyk(fusedSyncer)
        every { fusedSyncerSpy.start(startTimestamp) }.returns(true)
        estimator.setPrivateProperty("fusedSyncer", fusedSyncerSpy)

        val accelerometerFusedSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val accelerometerFusedSyncerSpy = spyk(accelerometerFusedSyncer)
        estimator.setPrivateProperty("accelerometerFusedSyncer", accelerometerFusedSyncerSpy)

        val attitudeSyncer: AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val attitudeSyncerSpy = spyk(attitudeSyncer)
        estimator.setPrivateProperty("attitudeSyncer", attitudeSyncerSpy)

        assertTrue(estimator.start(startTimestamp))
        assertTrue(estimator.running)

        verify { fusedProcessorSpy.reset() }
        verify(exactly = 1) { fusedSyncerSpy.start(startTimestamp) }
        verify { attitudeProcessorSpy wasNot Called }
        verify { accelerometerDoubleFusedProcessorSpy wasNot Called }
        verify { attitudeSyncerSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { doubleFusedProcessorSpy wasNot Called }
        verify { accelerometerFusedSyncerSpy wasNot Called }
    }

    @Test
    fun stop_stopsAllSyncers() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(context, initialLocation)

        val startTimestamp = System.nanoTime()

        val fusedSyncer: AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val fusedSyncerSpy = spyk(fusedSyncer)
        every { fusedSyncerSpy.start(startTimestamp) }.returns(true)
        estimator.setPrivateProperty("fusedSyncer", fusedSyncerSpy)

        val accelerometerFusedSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val accelerometerFusedSyncerSpy = spyk(accelerometerFusedSyncer)
        estimator.setPrivateProperty("accelerometerFusedSyncer", accelerometerFusedSyncerSpy)

        val attitudeSyncer: AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val attitudeSyncerSpy = spyk(attitudeSyncer)
        estimator.setPrivateProperty("attitudeSyncer", attitudeSyncerSpy)

        // stop
        estimator.stop()

        verify(exactly = 1) { fusedSyncerSpy.stop() }
        verify(exactly = 1) { accelerometerFusedSyncerSpy.stop() }
        verify(exactly = 1) { attitudeSyncerSpy.stop() }
    }

    @Test
    fun fusedSyncer_whenAccuracyChangedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(context, initialLocation)

        val fusedSyncer: AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val listener = fusedSyncer.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(fusedSyncer, SensorType.ABSOLUTE_ATTITUDE, SensorAccuracy.HIGH)
    }

    @Test
    fun fusedSyncer_whenAccuracyChangedAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            accuracyChangedListener = accuracyChangedListener
        )

        val fusedSyncer: AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val listener = fusedSyncer.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(fusedSyncer, SensorType.ABSOLUTE_ATTITUDE, SensorAccuracy.HIGH)

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
    fun fusedSyncer_whenBufferFilledAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(context, initialLocation)

        val fusedSyncer: AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val listener = fusedSyncer.bufferFilledListener
        requireNotNull(listener)

        // notify
        listener.onBufferFilled(fusedSyncer, SensorType.ABSOLUTE_ATTITUDE)
    }

    @Test
    fun fusedSyncer_whenBufferFilledAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            bufferFilledListener = bufferFilledListener
        )

        val fusedSyncer: AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val listener = fusedSyncer.bufferFilledListener
        requireNotNull(listener)

        // notify
        listener.onBufferFilled(fusedSyncer, SensorType.ABSOLUTE_ATTITUDE)

        // check
        verify(exactly = 1) {
            bufferFilledListener.onBufferFilled(
                estimator,
                SensorType.ABSOLUTE_ATTITUDE
            )
        }
    }

    @Test
    fun fusedSyncer_whenSyncedMeasurementUseDoubleFusedAttitudeAndNotProcessed_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            poseAvailableListener = poseAvailableListener
        )

        val fusedSyncer: AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val listener = fusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        every { doubleFusedProcessorSpy.process(any()) }.returns(false)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        // notify measurement
        val measurement = AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onSyncedMeasurements(fusedSyncer, measurement)

        // check
        verify(exactly = 1) { doubleFusedProcessorSpy.process(measurement) }
        verify { poseAvailableListener wasNot Called }
    }

    @Test
    fun fusedSyncer_whenSyncedMeasurementUseDoubleFusedAttitudeProcessedNotEstimatePoseTransformationAndNoListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = false
        )

        val fusedSyncer: AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val listener = fusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        every { doubleFusedProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        // notify measurement
        val measurement = AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onSyncedMeasurements(fusedSyncer, measurement)

        // check
        verify(exactly = 1) { doubleFusedProcessorSpy.process(measurement) }
        verify(inverse = true) { doubleFusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { doubleFusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { doubleFusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { doubleFusedProcessorSpy.initialEcefFrame }
    }

    @Test
    fun fusedSyncer_whenSyncedMeasurementUseDoubleFusedAttitudeProcessedEstimatePoseTransformationAndNoListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = true
        )

        val fusedSyncer: AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val listener = fusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("doubleFusedProcessor")
        requireNotNull(doubleFusedProcessor)
        val doubleFusedProcessorSpy = spyk(doubleFusedProcessor)
        every { doubleFusedProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty("doubleFusedProcessor", doubleFusedProcessorSpy)

        // notify measurement
        val measurement = AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onSyncedMeasurements(fusedSyncer, measurement)

        // check
        verify(exactly = 1) { doubleFusedProcessorSpy.process(measurement) }
        verify(exactly = 1) { doubleFusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { doubleFusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { doubleFusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { doubleFusedProcessorSpy.initialEcefFrame }
    }

    @Test
    fun fusedSyncer_whenSyncedMeasurementUseDoubleFusedAttitudeProcessedNotEstimatePoseTransformationAndListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = false,
            poseAvailableListener = poseAvailableListener
        )

        val fusedSyncer: AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val listener = fusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
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
        listener.onSyncedMeasurements(fusedSyncer, measurement)

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
    fun fusedSyncer_whenSyncedMeasurementUseDoubleFusedAttitudeProcessedEstimatePoseTransformationAndListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = true,
            poseAvailableListener = poseAvailableListener
        )

        val fusedSyncer: AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val listener = fusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val doubleFusedProcessor: DoubleFusedLocalPoseProcessor? =
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
        listener.onSyncedMeasurements(fusedSyncer, measurement)

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
    fun fusedSyncer_whenSyncedMeasurementNotUseDoubleFusedAttitudeAndNotProcessed_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = false,
            poseAvailableListener = poseAvailableListener
        )

        val fusedSyncer: AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val listener = fusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        every { fusedProcessorSpy.process(any()) }.returns(false)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        // notify measurement
        val measurement = AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onSyncedMeasurements(fusedSyncer, measurement)

        // check
        verify(exactly = 1) { fusedProcessorSpy.process(measurement) }
        verify { poseAvailableListener wasNot Called }
    }

    @Test
    fun fusedSyncer_whenSyncedMeasurementNotUseDoubleFusedAttitudeProcessedNotEstimatePoseTransformationAndNoListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = false,
            estimatePoseTransformation = false
        )

        val fusedSyncer: AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val listener = fusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        every { fusedProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        // notify measurement
        val measurement = AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onSyncedMeasurements(fusedSyncer, measurement)

        // check
        verify(exactly = 1) { fusedProcessorSpy.process(measurement) }
        verify(inverse = true) { fusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { fusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { fusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { fusedProcessorSpy.initialEcefFrame }
    }

    @Test
    fun fusedSyncer_whenSyncedMeasurementNotUseDoubleFusedAttitudeProcessedEstimatePoseTransformationAndNoListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = false,
            estimatePoseTransformation = true
        )

        val fusedSyncer: AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val listener = fusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val fusedProcessor: FusedLocalPoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        every { fusedProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        // notify measurement
        val measurement = AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onSyncedMeasurements(fusedSyncer, measurement)

        // check
        verify(exactly = 1) { fusedProcessorSpy.process(measurement) }
        verify(exactly = 1) { fusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { fusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { fusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { fusedProcessorSpy.initialEcefFrame }
    }

    @Test
    fun fusedSyncer_whenSyncedMeasurementNotUseDoubleFusedAttitudeProcessedNotEstimatePoseTransformationAndListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = false,
            estimatePoseTransformation = false,
            poseAvailableListener = poseAvailableListener
        )

        val fusedSyncer: AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val listener = fusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val fusedProcessor: FusedLocalPoseProcessor? =
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
        listener.onSyncedMeasurements(fusedSyncer, measurement)

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
    fun fusedSyncer_whenSyncedMeasurementNotUseDoubleFusedAttitudeProcessedEstimatePoseTransformationAndListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = false,
            estimatePoseTransformation = true,
            poseAvailableListener = poseAvailableListener
        )

        val fusedSyncer: AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val listener = fusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val fusedProcessor: FusedLocalPoseProcessor? =
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
        listener.onSyncedMeasurements(fusedSyncer, measurement)

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
    fun accelerometerFusedSyncer_whenAccuracyChangedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(context, initialLocation)

        val accelerometerFusedSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val listener = accelerometerFusedSyncer.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(
            accelerometerFusedSyncer,
            SensorType.ABSOLUTE_ATTITUDE,
            SensorAccuracy.HIGH
        )
    }

    @Test
    fun accelerometerFusedSyncer_whenAccuracyChangedAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            accuracyChangedListener = accuracyChangedListener
        )

        val accelerometerFusedSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val listener = accelerometerFusedSyncer.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(
            accelerometerFusedSyncer,
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
    fun accelerometerFusedSyncer_whenBufferFilledAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(context, initialLocation)

        val accelerometerFusedSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val listener = accelerometerFusedSyncer.bufferFilledListener
        requireNotNull(listener)

        // notify
        listener.onBufferFilled(accelerometerFusedSyncer, SensorType.ABSOLUTE_ATTITUDE)
    }

    @Test
    fun accelerometerFusedSyncer_whenBufferFilledAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            bufferFilledListener = bufferFilledListener
        )

        val accelerometerFusedSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val listener = accelerometerFusedSyncer.bufferFilledListener
        requireNotNull(listener)

        // notify
        listener.onBufferFilled(accelerometerFusedSyncer, SensorType.ABSOLUTE_ATTITUDE)

        // check
        verify(exactly = 1) {
            bufferFilledListener.onBufferFilled(
                estimator,
                SensorType.ABSOLUTE_ATTITUDE
            )
        }
    }

    @Test
    fun accelerometerFusedSyncer_whenSyncedMeasurementUseDoubleFusedAttitudeAndNotProcessed_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            poseAvailableListener = poseAvailableListener
        )

        val accelerometerFusedSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val listener = accelerometerFusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
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
        listener.onSyncedMeasurements(accelerometerFusedSyncer, measurement)

        // check
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.process(measurement) }
        verify { poseAvailableListener wasNot Called }
    }

    @Test
    fun accelerometerFusedSyncer_whenSyncedMeasurementUseDoubleFusedAttitudeProcessedNotEstimatePoseTransformationAndNoListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = false
        )

        val accelerometerFusedSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val listener = accelerometerFusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
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
        listener.onSyncedMeasurements(accelerometerFusedSyncer, measurement)

        // check
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.process(measurement) }
        verify(inverse = true) { accelerometerDoubleFusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.initialEcefFrame }
    }

    @Test
    fun accelerometerFusedSyncer_whenSyncedMeasurementUseDoubleFusedAttitudeProcessedEstimatePoseTransformationAndNoListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = true
        )

        val accelerometerFusedSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val listener = accelerometerFusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
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
        listener.onSyncedMeasurements(accelerometerFusedSyncer, measurement)

        // check
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.process(measurement) }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { accelerometerDoubleFusedProcessorSpy.initialEcefFrame }
    }

    @Test
    fun accelerometerFusedSyncer_whenSyncedMeasurementUseDoubleFusedAttitudeProcessedNotEstimatePoseTransformationAndListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = false,
            poseAvailableListener = poseAvailableListener
        )

        val accelerometerFusedSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val listener = accelerometerFusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
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
        listener.onSyncedMeasurements(accelerometerFusedSyncer, measurement)

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
    fun accelerometerFusedSyncer_whenSyncedMeasurementUseDoubleFusedAttitudeProcessedEstimatePoseTransformationAndListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = true,
            poseAvailableListener = poseAvailableListener
        )

        val accelerometerFusedSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val listener = accelerometerFusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val accelerometerDoubleFusedProcessor: AccelerometerDoubleFusedLocalPoseProcessor? =
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
        listener.onSyncedMeasurements(accelerometerFusedSyncer, measurement)

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
    fun accelerometerFusedSyncer_whenSyncedMeasurementNotUseDoubleFusedAttitudeAndNotProcessed_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = false,
            poseAvailableListener = poseAvailableListener
        )

        val accelerometerFusedSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val listener = accelerometerFusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
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
        listener.onSyncedMeasurements(accelerometerFusedSyncer, measurement)

        // check
        verify(exactly = 1) { accelerometerFusedProcessorSpy.process(measurement) }
        verify { poseAvailableListener wasNot Called }
    }

    @Test
    fun accelerometerFusedSyncer_whenSyncedMeasurementNotUseDoubleFusedAttitudeProcessedNotEstimatePoseTransformationAndNoListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = false,
            estimatePoseTransformation = false
        )

        val accelerometerFusedSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val listener = accelerometerFusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
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
        listener.onSyncedMeasurements(accelerometerFusedSyncer, measurement)

        // check
        verify(exactly = 1) { accelerometerFusedProcessorSpy.process(measurement) }
        verify(inverse = true) { accelerometerFusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.initialEcefFrame }
    }

    @Test
    fun accelerometerFusedSyncer_whenSyncedMeasurementNotUseDoubleFusedAttitudeProcessedEstimatePoseTransformationAndNoListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = false,
            estimatePoseTransformation = true
        )

        val accelerometerFusedSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val listener = accelerometerFusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        every { accelerometerFusedProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        // notify measurement
        val measurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onSyncedMeasurements(accelerometerFusedSyncer, measurement)

        // check
        verify(exactly = 1) { accelerometerFusedProcessorSpy.process(measurement) }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.poseTransformation }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.initialEcefFrame }
    }

    @Test
    fun accelerometerFusedSyncer_whenSyncedMeasurementNotUseDoubleFusedAttitudeProcessedNotEstimatePoseTransformationAndListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = false,
            estimatePoseTransformation = false,
            poseAvailableListener = poseAvailableListener
        )

        val accelerometerFusedSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val listener = accelerometerFusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
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
        listener.onSyncedMeasurements(accelerometerFusedSyncer, measurement)

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
    fun accelerometerFusedSyncer_whenSyncedMeasurementNotUseDoubleFusedAttitudeProcessedEstimatePoseTransformationAndListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = false,
            estimatePoseTransformation = true,
            poseAvailableListener = poseAvailableListener
        )

        val accelerometerFusedSyncer: AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val listener = accelerometerFusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val accelerometerFusedProcessor: AccelerometerFusedLocalPoseProcessor? =
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
        listener.onSyncedMeasurements(accelerometerFusedSyncer, measurement)

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
    fun attitudeSyncer_whenAccuracyChangedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(context, initialLocation)

        val attitudeSyncer: AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val listener = attitudeSyncer.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(
            attitudeSyncer,
            SensorType.ABSOLUTE_ATTITUDE,
            SensorAccuracy.HIGH
        )
    }

    @Test
    fun attitudeSyncer_whenAccuracyChangedAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            accuracyChangedListener = accuracyChangedListener
        )

        val attitudeSyncer: AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val listener = attitudeSyncer.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(
            attitudeSyncer,
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
    fun attitudeSyncer_whenBufferFilledAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(context, initialLocation)

        val attitudeSyncer: AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val listener = attitudeSyncer.bufferFilledListener
        requireNotNull(listener)

        // notify
        listener.onBufferFilled(attitudeSyncer, SensorType.ABSOLUTE_ATTITUDE)
    }

    @Test
    fun attitudeSyncer_whenBufferFilledAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            bufferFilledListener = bufferFilledListener
        )

        val attitudeSyncer: AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val listener = attitudeSyncer.bufferFilledListener
        requireNotNull(listener)

        // notify
        listener.onBufferFilled(attitudeSyncer, SensorType.ABSOLUTE_ATTITUDE)

        // check
        verify(exactly = 1) {
            bufferFilledListener.onBufferFilled(
                estimator,
                SensorType.ABSOLUTE_ATTITUDE
            )
        }
    }

    @Test
    fun attitudeSyncer_whenSyncedMeasurementAndNotProcessed_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            poseAvailableListener = poseAvailableListener
        )

        val attitudeSyncer: AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val listener = attitudeSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        every { attitudeProcessorSpy.process(any()) }.returns(false)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // notify measurement
        val measurement = AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement()
        listener.onSyncedMeasurements(attitudeSyncer, measurement)

        // check
        verify(exactly = 1) { attitudeProcessorSpy.process(measurement) }
        verify { poseAvailableListener wasNot Called }
    }

    @Test
    fun attitudeSyncer_whenSyncedMeasurementProcessedNotEstimatePoseTransformationAndNoListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = false
        )

        val attitudeSyncer: AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val listener = attitudeSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        every { attitudeProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // notify measurement
        val measurement = AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement()
        listener.onSyncedMeasurements(attitudeSyncer, measurement)

        // check
        verify(exactly = 1) { attitudeProcessorSpy.process(measurement) }
        verify(inverse = true) { attitudeProcessorSpy.poseTransformation }
        verify(exactly = 1) { attitudeProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { attitudeProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { attitudeProcessorSpy.initialEcefFrame }
    }

    @Test
    fun attitudeSyncer_whenSyncedMeasurementAttitudeProcessedEstimatePoseTransformationAndNoListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = true
        )

        val attitudeSyncer: AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val listener = attitudeSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        every { attitudeProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // notify measurement
        val measurement = AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement()
        listener.onSyncedMeasurements(attitudeSyncer, measurement)

        // check
        verify(exactly = 1) { attitudeProcessorSpy.process(measurement) }
        verify(exactly = 1) { attitudeProcessorSpy.poseTransformation }
        verify(exactly = 1) { attitudeProcessorSpy.currentEcefFrame }
        verify(exactly = 1) { attitudeProcessorSpy.previousEcefFrame }
        verify(exactly = 1) { attitudeProcessorSpy.initialEcefFrame }
    }

    @Test
    fun attitudeSyncer_whenSyncedMeasurementAttitudeProcessedNotEstimatePoseTransformationAndListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = false,
            poseAvailableListener = poseAvailableListener
        )

        val attitudeSyncer: AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val listener = attitudeSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
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
        listener.onSyncedMeasurements(attitudeSyncer, measurement)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialLocation = getLocation()
        val estimator = LocalPoseEstimator2(
            context,
            initialLocation,
            useDoubleFusedAttitudeProcessor = true,
            estimatePoseTransformation = true,
            poseAvailableListener = poseAvailableListener
        )

        val attitudeSyncer: AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val listener = attitudeSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val attitudeProcessor: AttitudeLocalPoseProcessor? =
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
        listener.onSyncedMeasurements(attitudeSyncer, measurement)

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