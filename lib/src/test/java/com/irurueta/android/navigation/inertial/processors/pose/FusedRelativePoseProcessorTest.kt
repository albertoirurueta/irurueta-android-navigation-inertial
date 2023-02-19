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
package com.irurueta.android.navigation.inertial.processors.pose

import com.irurueta.android.navigation.inertial.ENUtoNEDTriadConverter
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.estimators.pose.SpeedTriad
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.processors.attitude.BaseFusedGeomagneticAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.attitude.LeveledRelativeAttitudeProcessor
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.*
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator
import com.irurueta.navigation.inertial.estimators.NEDKinematicsEstimator
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.AccelerationUnit
import io.mockk.*
import org.junit.After
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class FusedRelativePoseProcessorTest {

    @After
    fun tearDown() {
        clearAllMocks()
    }

    @Test
    fun constructor_whenNoParameters_returnsExpectedValues() {
        val processor = FusedRelativePoseProcessor()

        // check
        assertEquals(SpeedTriad(), processor.initialSpeed)
        assertNull(processor.processorListener)
        assertNotNull(processor.poseTransformation)
        assertEquals(0.0, processor.timeIntervalSeconds, 0.0)
        assertEquals(0.0, processor.gx, 0.0)
        assertEquals(0.0, processor.gy, 0.0)
        assertEquals(0.0, processor.gz, 0.0)
        val gravity = AccelerationTriad()
        processor.getGravity(gravity)
        assertEquals(AccelerationTriad(), gravity)
        assertFalse(processor.useAccurateLevelingProcessor)
        assertTrue(processor.useAccurateRelativeGyroscopeAttitudeProcessor)
        assertTrue(processor.useIndirectAttitudeInterpolation)
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            processor.attitudeInterpolationValue,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            processor.attitudeIndirectInterpolationWeight,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            processor.attitudeOutlierThreshold,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            processor.attitudeOutlierPanicThreshold,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            processor.attitudePanicCounterThreshold
        )
    }

    @Test
    fun constructor_whenAllParameters_returnsExpectedValues() {
        val initialSpeed = getSpeed()
        val processorListener =
            mockk<BaseRelativePoseProcessor.OnProcessedListener>()
        val processor = FusedRelativePoseProcessor(initialSpeed, processorListener)

        // check
        assertSame(initialSpeed, processor.initialSpeed)
        assertSame(processorListener, processor.processorListener)
        assertNotNull(processor.poseTransformation)
        assertEquals(0.0, processor.timeIntervalSeconds, 0.0)
        assertEquals(0.0, processor.gx, 0.0)
        assertEquals(0.0, processor.gy, 0.0)
        assertEquals(0.0, processor.gz, 0.0)
        val gravity = AccelerationTriad()
        processor.getGravity(gravity)
        assertEquals(AccelerationTriad(), gravity)
        assertFalse(processor.useAccurateLevelingProcessor)
        assertTrue(processor.useAccurateRelativeGyroscopeAttitudeProcessor)
        assertTrue(processor.useIndirectAttitudeInterpolation)
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            processor.attitudeInterpolationValue,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            processor.attitudeIndirectInterpolationWeight,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            processor.attitudeOutlierThreshold,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            processor.attitudeOutlierPanicThreshold,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            processor.attitudePanicCounterThreshold
        )
    }

    @Test
    fun processorListener_setsExpectedValue() {
        val processor = FusedRelativePoseProcessor()

        // check default value
        assertNull(processor.processorListener)

        // set new value
        val processorListener = mockk<BaseRelativePoseProcessor.OnProcessedListener>()
        processor.processorListener = processorListener

        // check
        assertSame(processorListener, processor.processorListener)
    }

    @Test
    fun gx_returnsExpectedValue() {
        val processor = FusedRelativePoseProcessor()

        val attitudeProcessor: LeveledRelativeAttitudeProcessor? =
            processor.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextDouble()
        every { attitudeProcessorSpy.gx }.returns(gx)
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertEquals(gx, processor.gx, 0.0)

        verify(exactly = 1) { attitudeProcessorSpy.gx }
    }

    @Test
    fun gy_returnsExpectedValue() {
        val processor = FusedRelativePoseProcessor()

        val attitudeProcessor: LeveledRelativeAttitudeProcessor? =
            processor.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        val randomizer = UniformRandomizer()
        val gy = randomizer.nextDouble()
        every { attitudeProcessorSpy.gy }.returns(gy)
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertEquals(gy, processor.gy, 0.0)

        verify(exactly = 1) { attitudeProcessorSpy.gy }
    }

    @Test
    fun gz_returnsExpectedValue() {
        val processor = FusedRelativePoseProcessor()

        val attitudeProcessor: LeveledRelativeAttitudeProcessor? =
            processor.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        val randomizer = UniformRandomizer()
        val gz = randomizer.nextDouble()
        every { attitudeProcessorSpy.gz }.returns(gz)
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertEquals(gz, processor.gz, 0.0)

        verify(exactly = 1) { attitudeProcessorSpy.gz }
    }

    @Test
    fun getGravity_returnsExpectedValue() {
        val processor = FusedRelativePoseProcessor()

        val attitudeProcessor: LeveledRelativeAttitudeProcessor? =
            processor.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextDouble()
        val gy = randomizer.nextDouble()
        val gz = randomizer.nextDouble()
        every { attitudeProcessorSpy.getGravity(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as AccelerationTriad
            result.setValueCoordinates(gx, gy, gz)
        }
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val gravity = AccelerationTriad()
        processor.getGravity(gravity)

        // check
        assertEquals(AccelerationTriad(gx, gy, gz), gravity)

        verify(exactly = 1) { attitudeProcessorSpy.getGravity(gravity) }
    }

    @Test
    fun useAccurateRelativeGyroscopeAttitudeProcessor_setsExpectedValue() {
        val processor = FusedRelativePoseProcessor()

        val attitudeProcessor: LeveledRelativeAttitudeProcessor? =
            processor.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertTrue(processor.useAccurateRelativeGyroscopeAttitudeProcessor)

        // set new value
        processor.useAccurateRelativeGyroscopeAttitudeProcessor = false

        // check
        verify(exactly = 1) { attitudeProcessorSpy.useAccurateRelativeGyroscopeAttitudeProcessor }
        verify(exactly = 1) {
            attitudeProcessorSpy.useAccurateRelativeGyroscopeAttitudeProcessor = false
        }

        assertFalse(processor.useAccurateRelativeGyroscopeAttitudeProcessor)
    }

    @Test
    fun useIndirectAttitudeInterpolation_setsExpectedValue() {
        val processor = FusedRelativePoseProcessor()

        val attitudeProcessor: LeveledRelativeAttitudeProcessor? =
            processor.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertTrue(processor.useIndirectAttitudeInterpolation)

        // set new value
        processor.useIndirectAttitudeInterpolation = false

        // check
        verify(exactly = 1) { attitudeProcessorSpy.useIndirectInterpolation }
        verify(exactly = 1) {
            attitudeProcessorSpy.useIndirectInterpolation = false
        }

        assertFalse(processor.useIndirectAttitudeInterpolation)
    }

    @Test
    fun attitudeInterpolationValue_setsExpectedValue() {
        val processor = FusedRelativePoseProcessor()

        val attitudeProcessor: LeveledRelativeAttitudeProcessor? =
            processor.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            processor.attitudeInterpolationValue,
            0.0
        )

        // set nw value
        val randomizer = UniformRandomizer()
        val attitudeInterpolationValue = randomizer.nextDouble()
        processor.attitudeInterpolationValue = attitudeInterpolationValue

        // check
        verify(exactly = 1) { attitudeProcessorSpy.interpolationValue }
        verify(exactly = 1) {
            attitudeProcessorSpy.interpolationValue = attitudeInterpolationValue
        }

        assertEquals(attitudeInterpolationValue, processor.attitudeInterpolationValue, 0.0)
    }

    @Test
    fun attitudeIndirectInterpolationWeight_setsExpectedValue() {
        val processor = FusedRelativePoseProcessor()

        val attitudeProcessor: LeveledRelativeAttitudeProcessor? =
            processor.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            processor.attitudeIndirectInterpolationWeight,
            0.0
        )

        // set nw value
        val randomizer = UniformRandomizer()
        val attitudeIndirectInterpolationWeight = randomizer.nextDouble()
        processor.attitudeIndirectInterpolationWeight = attitudeIndirectInterpolationWeight

        // check
        verify(exactly = 1) { attitudeProcessorSpy.indirectInterpolationWeight }
        verify(exactly = 1) {
            attitudeProcessorSpy.indirectInterpolationWeight = attitudeIndirectInterpolationWeight
        }

        assertEquals(
            attitudeIndirectInterpolationWeight,
            processor.attitudeIndirectInterpolationWeight,
            0.0
        )
    }

    @Test
    fun attitudeOutlierThreshold_setsExpectedValue() {
        val processor = FusedRelativePoseProcessor()

        val attitudeProcessor: LeveledRelativeAttitudeProcessor? =
            processor.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            processor.attitudeOutlierThreshold,
            0.0
        )

        // set nw value
        val randomizer = UniformRandomizer()
        val attitudeOutlierThreshold = randomizer.nextDouble()
        processor.attitudeOutlierThreshold = attitudeOutlierThreshold

        // check
        verify(exactly = 1) { attitudeProcessorSpy.outlierThreshold }
        verify(exactly = 1) {
            attitudeProcessorSpy.outlierThreshold = attitudeOutlierThreshold
        }

        assertEquals(
            attitudeOutlierThreshold,
            processor.attitudeOutlierThreshold,
            0.0
        )
    }

    @Test
    fun attitudeOutlierPanicThreshold_setsExpectedValue() {
        val processor = FusedRelativePoseProcessor()

        val attitudeProcessor: LeveledRelativeAttitudeProcessor? =
            processor.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            processor.attitudeOutlierPanicThreshold,
            0.0
        )

        // set nw value
        val randomizer = UniformRandomizer()
        val attitudeOutlierPanicThreshold = randomizer.nextDouble()
        processor.attitudeOutlierPanicThreshold = attitudeOutlierPanicThreshold

        // check
        verify(exactly = 1) { attitudeProcessorSpy.outlierPanicThreshold }
        verify(exactly = 1) {
            attitudeProcessorSpy.outlierPanicThreshold = attitudeOutlierPanicThreshold
        }

        assertEquals(
            attitudeOutlierPanicThreshold,
            processor.attitudeOutlierPanicThreshold,
            0.0
        )
    }

    @Test
    fun attitudePanicCounterThreshold_setsExpectedValue() {
        val processor = FusedRelativePoseProcessor()

        val attitudeProcessor: LeveledRelativeAttitudeProcessor? =
            processor.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            processor.attitudePanicCounterThreshold
        )

        // set nw value
        val randomizer = UniformRandomizer()
        val attitudePanicCounterThreshold = randomizer.nextInt(1, 100)
        processor.attitudePanicCounterThreshold = attitudePanicCounterThreshold

        // check
        verify(exactly = 1) { attitudeProcessorSpy.panicCounterThreshold }
        verify(exactly = 1) {
            attitudeProcessorSpy.panicCounterThreshold = attitudePanicCounterThreshold
        }

        assertEquals(
            attitudePanicCounterThreshold,
            processor.attitudePanicCounterThreshold
        )
    }

    @Test
    fun reset_initializesInternalParameters() {
        val processor = FusedRelativePoseProcessor()

        val attitudeProcessor: LeveledRelativeAttitudeProcessor? =
            processor.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        setPrivateProperty(
            BaseRelativePoseProcessor::class,
            processor,
            "initialized",
            true
        )
        val previousTimestamp = System.nanoTime()
        setPrivateProperty(
            BaseRelativePoseProcessor::class,
            processor,
            "previousTimestamp",
            previousTimestamp
        )
        setPrivateProperty(
            BaseRelativePoseProcessor::class,
            processor,
            "timeIntervalSeconds",
            TIME_INTERVAL_SECONDS
        )

        // check initial values
        val initialized1: Boolean? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "initialized")
        requireNotNull(initialized1)
        assertTrue(initialized1)

        val previousTimestamp1: Long? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp1)
        assertEquals(previousTimestamp, previousTimestamp1)

        val timeIntervalSeconds1: Double? =
            getPrivateProperty(
                BaseRelativePoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds1)
        assertEquals(TIME_INTERVAL_SECONDS, timeIntervalSeconds1, 0.0)

        // reset
        processor.reset()

        // check
        val initialized2: Boolean? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "initialized")
        requireNotNull(initialized2)
        assertFalse(initialized2)

        val previousTimestamp2: Long? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp2)
        assertEquals(-1L, previousTimestamp2)

        val timeIntervalSeconds2: Double? =
            getPrivateProperty(
                BaseRelativePoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds2)
        assertEquals(0.0, timeIntervalSeconds2, 0.0)

        verify(exactly = 1) { attitudeProcessorSpy.reset() }
    }

    @Test
    fun process_whenEmpty_returnsFalse() {
        val processor = FusedRelativePoseProcessor()

        val syncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement()
        assertFalse(processor.process(syncedMeasurement))
    }

    @Test
    fun process_whenNoAccelerometerMeasurement_returnsFalse() {
        val processor = FusedRelativePoseProcessor()

        val syncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement = null,
            gravityMeasurement = GravitySensorMeasurement(),
            gyroscopeMeasurement = GyroscopeSensorMeasurement(),
            timestamp = System.nanoTime()
        )
        assertFalse(processor.process(syncedMeasurement))
    }

    @Test
    fun process_whenNoGravityMeasurement_returnsFalse() {
        val processor = FusedRelativePoseProcessor()

        val syncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement = AccelerometerSensorMeasurement(),
            gravityMeasurement = null,
            gyroscopeMeasurement = GyroscopeSensorMeasurement(),
            timestamp = System.nanoTime()
        )
        assertFalse(processor.process(syncedMeasurement))
    }

    @Test
    fun process_whenNoGyroscopeMeasurement_returnsFalse() {
        val processor = FusedRelativePoseProcessor()

        val syncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement = AccelerometerSensorMeasurement(),
            gravityMeasurement = GravitySensorMeasurement(),
            gyroscopeMeasurement = null,
            timestamp = System.nanoTime()
        )
        assertFalse(processor.process(syncedMeasurement))
    }

    @Test
    fun process_whenAttitudeNotProcessed_returnsFalse() {
        val processor = FusedRelativePoseProcessor()

        val timestamp = System.nanoTime()
        val nedAttitude = getAttitude()

        val position = getNEDPosition()
        val c = CoordinateTransformation(
            nedAttitude,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val nedFrame = NEDFrame(position, NEDVelocity(), c)
        val kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(
            TIME_INTERVAL_SECONDS,
            nedFrame,
            nedFrame
        )

        val accelerometerEnu = AccelerationTriad()
        ENUtoNEDTriadConverter.convert(
            kinematics.fx,
            kinematics.fy,
            kinematics.fz,
            accelerometerEnu
        )

        val gyroscopeEnu = AngularSpeedTriad()
        ENUtoNEDTriadConverter.convert(
            kinematics.angularRateX,
            kinematics.angularRateY,
            kinematics.angularRateZ,
            gyroscopeEnu
        )

        val nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(nedFrame)
        val enuGravity = AccelerationTriad()
        ENUtoNEDTriadConverter.convert(nedGravity.gn, nedGravity.ge, nedGravity.gd, enuGravity)

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax = accelerometerEnu.valueX.toFloat(),
            ay = accelerometerEnu.valueY.toFloat(),
            az = accelerometerEnu.valueZ.toFloat(),
            timestamp = timestamp
        )

        val gravityMeasurement = GravitySensorMeasurement(
            gx = enuGravity.valueX.toFloat(),
            gy = enuGravity.valueY.toFloat(),
            gz = enuGravity.valueZ.toFloat()
        )

        val gyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx = gyroscopeEnu.valueX.toFloat(),
            wy = gyroscopeEnu.valueY.toFloat(),
            wz = gyroscopeEnu.valueZ.toFloat(),
            timestamp = timestamp
        )
        val syncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gravityMeasurement,
            gyroscopeMeasurement,
            timestamp
        )

        val attitudeProcessor: LeveledRelativeAttitudeProcessor? =
            processor.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        every { attitudeProcessorSpy.process(any(), any(), any()) }.returns(false)
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // check
        val gravity: AccelerationTriad? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "gravity")
        requireNotNull(gravity)
        assertEquals(AccelerationTriad(), gravity)

        // process
        assertFalse(processor.process(syncedMeasurement))

        // check
        verify(exactly = 1) {
            attitudeProcessorSpy.process(
                gravityMeasurement,
                gyroscopeMeasurement,
                timestamp
            )
        }

        val currentAttitude: Quaternion? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "currentAttitude")
        requireNotNull(currentAttitude)

        assertEquals(Quaternion(), currentAttitude)

        assertEquals(AccelerationTriad(), gravity)
    }

    @Test
    fun process_whenAttitudeProcessedAndTimeIntervalNotProcessed_returnsFalse() {
        val processor = FusedRelativePoseProcessor()

        val timestamp = System.nanoTime()
        val nedAttitude = getAttitude()

        val position = getNEDPosition()
        val c = CoordinateTransformation(
            nedAttitude,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val nedFrame = NEDFrame(position, NEDVelocity(), c)
        val kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(
            TIME_INTERVAL_SECONDS,
            nedFrame,
            nedFrame
        )

        val accelerometerEnu = AccelerationTriad()
        ENUtoNEDTriadConverter.convert(
            kinematics.fx,
            kinematics.fy,
            kinematics.fz,
            accelerometerEnu
        )

        val gyroscopeEnu = AngularSpeedTriad()
        ENUtoNEDTriadConverter.convert(
            kinematics.angularRateX,
            kinematics.angularRateY,
            kinematics.angularRateZ,
            gyroscopeEnu
        )

        val nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(nedFrame)
        val enuGravity = AccelerationTriad()
        ENUtoNEDTriadConverter.convert(nedGravity.gn, nedGravity.ge, nedGravity.gd, enuGravity)

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax = accelerometerEnu.valueX.toFloat(),
            ay = accelerometerEnu.valueY.toFloat(),
            az = accelerometerEnu.valueZ.toFloat(),
            timestamp = timestamp
        )

        val gravityMeasurement = GravitySensorMeasurement(
            gx = enuGravity.valueX.toFloat(),
            gy = enuGravity.valueY.toFloat(),
            gz = enuGravity.valueZ.toFloat()
        )

        val gyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx = gyroscopeEnu.valueX.toFloat(),
            wy = gyroscopeEnu.valueY.toFloat(),
            wz = gyroscopeEnu.valueZ.toFloat(),
            timestamp = timestamp
        )
        val syncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gravityMeasurement,
            gyroscopeMeasurement,
            timestamp
        )

        val attitudeProcessor: LeveledRelativeAttitudeProcessor? =
            processor.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        every { attitudeProcessorSpy.process(any(), any(), any()) }.returns(true)
        every { attitudeProcessorSpy.fusedAttitude }.returns(nedAttitude)
        every { attitudeProcessorSpy.getGravity(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as AccelerationTriad
            result.setValueCoordinatesAndUnit(
                nedGravity.gn,
                nedGravity.ge,
                nedGravity.gd,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
        }
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // check
        val gravity: AccelerationTriad? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "gravity")
        requireNotNull(gravity)
        assertEquals(AccelerationTriad(), gravity)
        val previousTimestamp1: Long? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp1)
        assertEquals(-1L, previousTimestamp1)

        val timeIntervalSeconds1: Double? =
            getPrivateProperty(
                BaseRelativePoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds1)
        assertEquals(0.0, timeIntervalSeconds1, 0.0)

        // process
        assertFalse(processor.process(syncedMeasurement))

        // check
        verify(exactly = 1) {
            attitudeProcessorSpy.process(
                gravityMeasurement,
                gyroscopeMeasurement,
                timestamp
            )
        }

        val currentAttitude: Quaternion? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "currentAttitude")
        requireNotNull(currentAttitude)

        assertEquals(nedAttitude, currentAttitude)

        assertEquals(AccelerationTriad(nedGravity.gn, nedGravity.ge, nedGravity.gd), gravity)

        val previousTimestamp2: Long? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp2)
        assertEquals(timestamp, previousTimestamp2)

        val timeIntervalSeconds2: Double? =
            getPrivateProperty(
                BaseRelativePoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds2)
        assertEquals(0.0, timeIntervalSeconds2, 0.0)
    }

    @Test
    fun process_whenAttitudeAndTimeIntervalProcessed_returnsTrue() {
        val processor = FusedRelativePoseProcessor()

        val timestamp = System.nanoTime()
        val nedAttitude = getAttitude()

        val position = getNEDPosition()
        val c = CoordinateTransformation(
            nedAttitude,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val nedFrame = NEDFrame(position, NEDVelocity(), c)
        // define kinematics to keep device static with current frame (same position, speed and
        // attitude)
        val kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(
            TIME_INTERVAL_SECONDS,
            nedFrame,
            nedFrame
        )

        val accelerometerEnu = AccelerationTriad()
        ENUtoNEDTriadConverter.convert(
            kinematics.fx,
            kinematics.fy,
            kinematics.fz,
            accelerometerEnu
        )

        val gyroscopeEnu = AngularSpeedTriad()
        ENUtoNEDTriadConverter.convert(
            kinematics.angularRateX,
            kinematics.angularRateY,
            kinematics.angularRateZ,
            gyroscopeEnu
        )

        val nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(nedFrame)
        val enuGravity = AccelerationTriad()
        ENUtoNEDTriadConverter.convert(nedGravity.gn, nedGravity.ge, nedGravity.gd, enuGravity)

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax = accelerometerEnu.valueX.toFloat(),
            ay = accelerometerEnu.valueY.toFloat(),
            az = accelerometerEnu.valueZ.toFloat(),
            timestamp = timestamp
        )

        val gravityMeasurement = GravitySensorMeasurement(
            gx = enuGravity.valueX.toFloat(),
            gy = enuGravity.valueY.toFloat(),
            gz = enuGravity.valueZ.toFloat()
        )

        val gyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx = gyroscopeEnu.valueX.toFloat(),
            wy = gyroscopeEnu.valueY.toFloat(),
            wz = gyroscopeEnu.valueZ.toFloat(),
            timestamp = timestamp
        )
        val syncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gravityMeasurement,
            gyroscopeMeasurement,
            timestamp
        )

        val previousTimestamp = timestamp - TIME_INTERVAL_NANOS
        setPrivateProperty(
            BaseRelativePoseProcessor::class,
            processor,
            "previousTimestamp",
            previousTimestamp
        )

        val attitudeProcessor: LeveledRelativeAttitudeProcessor? =
            processor.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        every { attitudeProcessorSpy.process(any(), any(), any()) }.returns(true)
        every { attitudeProcessorSpy.fusedAttitude }.returns(nedAttitude)
        every { attitudeProcessorSpy.getGravity(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as AccelerationTriad
            result.setValueCoordinatesAndUnit(
                nedGravity.gn,
                nedGravity.ge,
                nedGravity.gd,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
        }
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // check
        val gravity: AccelerationTriad? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "gravity")
        requireNotNull(gravity)
        assertEquals(AccelerationTriad(), gravity)
        val previousTimestamp1: Long? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp1)
        assertEquals(previousTimestamp, previousTimestamp1)

        val timeIntervalSeconds1: Double? =
            getPrivateProperty(
                BaseRelativePoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds1)
        assertEquals(0.0, timeIntervalSeconds1, 0.0)

        // process
        assertTrue(processor.process(syncedMeasurement))

        // check
        verify(exactly = 1) {
            attitudeProcessorSpy.process(
                gravityMeasurement,
                gyroscopeMeasurement,
                timestamp
            )
        }

        val currentAttitude: Quaternion? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "currentAttitude")
        requireNotNull(currentAttitude)
        assertEquals(nedAttitude, currentAttitude)

        assertEquals(AccelerationTriad(nedGravity.gn, nedGravity.ge, nedGravity.gd), gravity)

        val previousTimestamp2: Long? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp2)
        assertEquals(timestamp, previousTimestamp2)

        val timeIntervalSeconds2: Double? =
            getPrivateProperty(
                BaseRelativePoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds2)
        assertEquals(TIME_INTERVAL_SECONDS, timeIntervalSeconds2, 0.0)

        val transformation = processor.poseTransformation
        val transformationRotation = transformation.rotation.toQuaternion()
        transformationRotation.normalize()
        val conversionRotation = ENUtoNEDTriadConverter.conversionRotation
        val transformationRotation2 = nedAttitude.multiplyAndReturnNew(conversionRotation)
        assertTrue(transformationRotation.equals(transformationRotation2))
        assertArrayEquals(DoubleArray(3), transformation.translation, VERY_LARGE_ABSOLUTE_ERROR)
    }

    @Test
    fun process_whenAttitudeAndTimeIntervalProcessedWithListener_returnsTrueAndNotifies() {
        val processorListener =
            mockk<BaseRelativePoseProcessor.OnProcessedListener>(relaxUnitFun = true)
        val processor = FusedRelativePoseProcessor(processorListener = processorListener)

        val timestamp = System.nanoTime()
        val nedAttitude = getAttitude()

        val position = getNEDPosition()
        val c = CoordinateTransformation(
            nedAttitude,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val nedFrame = NEDFrame(position, NEDVelocity(), c)
        // define kinematics to keep device static with current frame (same position, speed and
        // attitude)
        val kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(
            TIME_INTERVAL_SECONDS,
            nedFrame,
            nedFrame
        )

        val accelerometerEnu = AccelerationTriad()
        ENUtoNEDTriadConverter.convert(
            kinematics.fx,
            kinematics.fy,
            kinematics.fz,
            accelerometerEnu
        )

        val gyroscopeEnu = AngularSpeedTriad()
        ENUtoNEDTriadConverter.convert(
            kinematics.angularRateX,
            kinematics.angularRateY,
            kinematics.angularRateZ,
            gyroscopeEnu
        )

        val nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(nedFrame)
        val enuGravity = AccelerationTriad()
        ENUtoNEDTriadConverter.convert(nedGravity.gn, nedGravity.ge, nedGravity.gd, enuGravity)

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax = accelerometerEnu.valueX.toFloat(),
            ay = accelerometerEnu.valueY.toFloat(),
            az = accelerometerEnu.valueZ.toFloat(),
            timestamp = timestamp
        )

        val gravityMeasurement = GravitySensorMeasurement(
            gx = enuGravity.valueX.toFloat(),
            gy = enuGravity.valueY.toFloat(),
            gz = enuGravity.valueZ.toFloat()
        )

        val gyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx = gyroscopeEnu.valueX.toFloat(),
            wy = gyroscopeEnu.valueY.toFloat(),
            wz = gyroscopeEnu.valueZ.toFloat(),
            timestamp = timestamp
        )
        val syncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gravityMeasurement,
            gyroscopeMeasurement,
            timestamp
        )

        val previousTimestamp = timestamp - TIME_INTERVAL_NANOS
        setPrivateProperty(
            BaseRelativePoseProcessor::class,
            processor,
            "previousTimestamp",
            previousTimestamp
        )

        val attitudeProcessor: LeveledRelativeAttitudeProcessor? =
            processor.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        every { attitudeProcessorSpy.process(any(), any(), any()) }.returns(true)
        every { attitudeProcessorSpy.fusedAttitude }.returns(nedAttitude)
        every { attitudeProcessorSpy.getGravity(any()) }.answers { answer ->
            val result = answer.invocation.args[0] as AccelerationTriad
            result.setValueCoordinatesAndUnit(
                nedGravity.gn,
                nedGravity.ge,
                nedGravity.gd,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
        }
        processor.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // check
        val gravity: AccelerationTriad? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "gravity")
        requireNotNull(gravity)
        assertEquals(AccelerationTriad(), gravity)
        val previousTimestamp1: Long? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp1)
        assertEquals(previousTimestamp, previousTimestamp1)

        val timeIntervalSeconds1: Double? =
            getPrivateProperty(
                BaseRelativePoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds1)
        assertEquals(0.0, timeIntervalSeconds1, 0.0)

        // process
        assertTrue(processor.process(syncedMeasurement))

        // check
        verify(exactly = 1) {
            attitudeProcessorSpy.process(
                gravityMeasurement,
                gyroscopeMeasurement,
                timestamp
            )
        }

        val currentAttitude: Quaternion? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "currentAttitude")
        requireNotNull(currentAttitude)
        assertEquals(nedAttitude, currentAttitude)

        assertEquals(AccelerationTriad(nedGravity.gn, nedGravity.ge, nedGravity.gd), gravity)

        val previousTimestamp2: Long? =
            getPrivateProperty(BaseRelativePoseProcessor::class, processor, "previousTimestamp")
        requireNotNull(previousTimestamp2)
        assertEquals(timestamp, previousTimestamp2)

        val timeIntervalSeconds2: Double? =
            getPrivateProperty(
                BaseRelativePoseProcessor::class,
                processor,
                "timeIntervalSeconds"
            )
        requireNotNull(timeIntervalSeconds2)
        assertEquals(TIME_INTERVAL_SECONDS, timeIntervalSeconds2, 0.0)

        val transformation = processor.poseTransformation
        val transformationRotation = transformation.rotation.toQuaternion()
        transformationRotation.normalize()
        val conversionRotation = ENUtoNEDTriadConverter.conversionRotation
        val transformationRotation2 = nedAttitude.multiplyAndReturnNew(conversionRotation)
        assertTrue(transformationRotation.equals(transformationRotation2))
        assertArrayEquals(DoubleArray(3), transformation.translation, VERY_LARGE_ABSOLUTE_ERROR)

        verify(exactly = 1) { processorListener.onProcessed(processor, timestamp, transformation) }
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

        const val MIN_SPEED = -1.0
        const val MAX_SPEED = 1.0

        const val TIME_INTERVAL_SECONDS = 0.01

        const val TIME_INTERVAL_NANOS = 10_000_000L

        const val VERY_LARGE_ABSOLUTE_ERROR = 1e-3

        fun getSpeed(): SpeedTriad {
            val randomizer = UniformRandomizer()
            val vx = randomizer.nextDouble(MIN_SPEED, MAX_SPEED)
            val vy = randomizer.nextDouble(MIN_SPEED, MAX_SPEED)
            val vz = randomizer.nextDouble(MIN_SPEED, MAX_SPEED)
            return SpeedTriad(vx, vy, vz)
        }

        fun getAttitude(): Quaternion {
            val randomizer = UniformRandomizer()
            val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))

            return Quaternion(roll, pitch, yaw)
        }

        fun getNEDPosition(): NEDPosition {
            val randomizer = UniformRandomizer()
            val latitudeDegrees = randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES)
            val longitudeDegrees =
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES)
            val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

            return NEDPosition(
                Math.toRadians(latitudeDegrees),
                Math.toRadians(longitudeDegrees),
                height
            )
        }
    }
}