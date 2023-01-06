package com.irurueta.android.navigation.inertial.processors

import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator
import com.irurueta.statistics.UniformRandomizer
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test

class RelativeGyroscopeAttitudeProcessorTest {

    @Test
    fun constructor_whenNoParameters_setsDefaultValues() {
        val processor = RelativeGyroscopeAttitudeProcessor()

        assertNull(processor.processorListener)
        assertEquals(Quaternion(), processor.attitude)
        assertEquals(0.0, processor.averageTimeInterval, 0.0)
    }

    @Test
    fun constructor_whenListener_setsExpectedValues() {
        val listener = mockk<BaseRelativeGyroscopeAttitudeProcessor.OnProcessedListener>()
        val processor = RelativeGyroscopeAttitudeProcessor(listener)

        assertSame(listener, processor.processorListener)
        assertEquals(Quaternion(), processor.attitude)
        assertEquals(0.0, processor.averageTimeInterval, 0.0)
    }

    @Test
    fun processorListener_setsExpectedValue() {
        val processor = RelativeGyroscopeAttitudeProcessor()

        // check default value
        assertNull(processor.processorListener)

        // set new value
        val listener = mockk<BaseRelativeGyroscopeAttitudeProcessor.OnProcessedListener>()
        processor.processorListener = listener

        // check
        assertSame(listener, processor.processorListener)
    }

    @Test
    fun process_whenFirstMeasurement_setsInitialTimestampInTimeIntervalEstimator() {
        val listener =
            mockk<BaseRelativeGyroscopeAttitudeProcessor.OnProcessedListener>(relaxUnitFun = true)
        val processor = RelativeGyroscopeAttitudeProcessor(listener)

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "timeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        assertEquals(0, timeIntervalEstimator.numberOfProcessedSamples)
        assertNull(timeIntervalEstimator.lastTimestamp)

        val initialTimestamp1: Long? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "initialTimestamp"
        )
        requireNotNull(initialTimestamp1)
        assertEquals(0L, initialTimestamp1)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val measurement = GyroscopeSensorMeasurement(wx, wy, wz, null, null, null, timestamp)
        assertFalse(processor.process(measurement))

        // check
        val initialTimestamp2: Long? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "initialTimestamp"
        )
        requireNotNull(initialTimestamp2)
        assertEquals(timestamp, initialTimestamp2)

        assertEquals(Quaternion(), processor.attitude)
        assertEquals(0.0, processor.averageTimeInterval, 0.0)

        verify { listener wasNot Called }
    }

    @Test
    fun process_whenNotFirstMeasurement_setsExpectedAttitudeAndNotifies() {
        val listener =
            mockk<BaseRelativeGyroscopeAttitudeProcessor.OnProcessedListener>(relaxUnitFun = true)
        val processor = RelativeGyroscopeAttitudeProcessor(listener)

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "timeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.numberOfProcessedSamples }.returns(1)
        every { timeIntervalEstimatorSpy.averageTimeInterval }.returns(INTERVAL_SECONDS)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "timeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        val timestamp = System.nanoTime()
        val initialTimestamp = timestamp - INTERVAL_NANOS
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "initialTimestamp",
            initialTimestamp
        )

        val triad: AngularSpeedTriad? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "triad"
        )
        requireNotNull(triad)
        val triadSpy = spyk(triad)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "triad",
            triadSpy
        )

        val deltaAttitude: Quaternion? = processor.getPrivateProperty("deltaAttitude")
        requireNotNull(deltaAttitude)
        val deltaAttitudeSpy = spyk(deltaAttitude)
        processor.setPrivateProperty("deltaAttitude", deltaAttitudeSpy)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val measurement = GyroscopeSensorMeasurement(
            wy,
            wx,
            -wz,
            null,
            null,
            null,
            timestamp,
            SensorAccuracy.HIGH
        )
        assertTrue(processor.process(measurement))

        // check
        verify(exactly = 1) {
            triadSpy.setValueCoordinates(
                wx.toDouble(),
                wy.toDouble(),
                wz.toDouble()
            )
        }

        val expectedRoll = wx.toDouble() * INTERVAL_SECONDS
        val expectedPitch = wy.toDouble() * INTERVAL_SECONDS
        val expectedYaw = wz.toDouble() * INTERVAL_SECONDS
        verify(exactly = 1) {
            deltaAttitudeSpy.setFromEulerAngles(
                expectedRoll,
                expectedPitch,
                expectedYaw
            )
        }

        val expectedAttitude = Quaternion(expectedRoll, expectedPitch, expectedYaw)
        assertEquals(expectedAttitude, processor.attitude)

        verify(exactly = 1) {
            listener.onProcessed(
                processor,
                expectedAttitude,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun process_whenNotFirstMeasurementWithBias_setsExpectedAttitudeAndNotifies() {
        val listener =
            mockk<BaseRelativeGyroscopeAttitudeProcessor.OnProcessedListener>(relaxUnitFun = true)
        val processor = RelativeGyroscopeAttitudeProcessor(listener)

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "timeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.numberOfProcessedSamples }.returns(1)
        every { timeIntervalEstimatorSpy.averageTimeInterval }.returns(INTERVAL_SECONDS)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "timeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        val timestamp = System.nanoTime()
        val initialTimestamp = timestamp - INTERVAL_NANOS
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "initialTimestamp",
            initialTimestamp
        )

        val triad: AngularSpeedTriad? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "triad"
        )
        requireNotNull(triad)
        val triadSpy = spyk(triad)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "triad",
            triadSpy
        )

        val deltaAttitude: Quaternion? = processor.getPrivateProperty("deltaAttitude")
        requireNotNull(deltaAttitude)
        val deltaAttitudeSpy = spyk(deltaAttitude)
        processor.setPrivateProperty("deltaAttitude", deltaAttitudeSpy)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val measurement =
            GyroscopeSensorMeasurement(wy, wx, -wz, by, bx, -bz, timestamp, SensorAccuracy.HIGH)
        assertTrue(processor.process(measurement))

        // check
        verify(exactly = 1) {
            triadSpy.setValueCoordinates(
                wx.toDouble() - bx.toDouble(),
                wy.toDouble() - by.toDouble(),
                wz.toDouble() - bz.toDouble()
            )
        }

        val expectedRoll = (wx.toDouble() - bx.toDouble()) * INTERVAL_SECONDS
        val expectedPitch = (wy.toDouble() - by.toDouble()) * INTERVAL_SECONDS
        val expectedYaw = (wz.toDouble() - bz.toDouble()) * INTERVAL_SECONDS
        verify(exactly = 1) {
            deltaAttitudeSpy.setFromEulerAngles(
                expectedRoll,
                expectedPitch,
                expectedYaw
            )
        }

        val expectedAttitude = Quaternion(expectedRoll, expectedPitch, expectedYaw)
        assertEquals(expectedAttitude, processor.attitude)

        verify(exactly = 1) {
            listener.onProcessed(
                processor,
                expectedAttitude,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun process_whenProvidedTimestamp_setsExpectedAttitudeAndNotifies() {
        val listener =
            mockk<BaseRelativeGyroscopeAttitudeProcessor.OnProcessedListener>(relaxUnitFun = true)
        val processor = RelativeGyroscopeAttitudeProcessor(listener)

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "timeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        every { timeIntervalEstimatorSpy.numberOfProcessedSamples }.returns(1)
        every { timeIntervalEstimatorSpy.averageTimeInterval }.returns(INTERVAL_SECONDS)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "timeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        val timestamp = System.nanoTime()
        val initialTimestamp = timestamp - INTERVAL_NANOS
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "initialTimestamp",
            initialTimestamp
        )

        val triad: AngularSpeedTriad? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "triad"
        )
        requireNotNull(triad)
        val triadSpy = spyk(triad)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "triad",
            triadSpy
        )

        val deltaAttitude: Quaternion? = processor.getPrivateProperty("deltaAttitude")
        requireNotNull(deltaAttitude)
        val deltaAttitudeSpy = spyk(deltaAttitude)
        processor.setPrivateProperty("deltaAttitude", deltaAttitudeSpy)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val measurement =
            GyroscopeSensorMeasurement(wy, wx, -wz, by, bx, -bz, timestamp, SensorAccuracy.HIGH)
        assertTrue(processor.process(measurement, timestamp))

        // check
        verify(exactly = 1) {
            triadSpy.setValueCoordinates(
                wx.toDouble() - bx.toDouble(),
                wy.toDouble() - by.toDouble(),
                wz.toDouble() - bz.toDouble()
            )
        }

        val expectedRoll = (wx.toDouble() - bx.toDouble()) * INTERVAL_SECONDS
        val expectedPitch = (wy.toDouble() - by.toDouble()) * INTERVAL_SECONDS
        val expectedYaw = (wz.toDouble() - bz.toDouble()) * INTERVAL_SECONDS
        verify(exactly = 1) {
            deltaAttitudeSpy.setFromEulerAngles(
                expectedRoll,
                expectedPitch,
                expectedYaw
            )
        }

        val expectedAttitude = Quaternion(expectedRoll, expectedPitch, expectedYaw)
        assertEquals(expectedAttitude, processor.attitude)

        verify(exactly = 1) {
            listener.onProcessed(
                processor,
                expectedAttitude,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun reset_restoresInitialState() {
        val processor = RelativeGyroscopeAttitudeProcessor()

        val timeIntervalEstimator: TimeIntervalEstimator? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "timeIntervalEstimator"
        )
        requireNotNull(timeIntervalEstimator)
        val timeIntervalEstimatorSpy = spyk(timeIntervalEstimator)
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "timeIntervalEstimator",
            timeIntervalEstimatorSpy
        )

        val initialTimestamp1 = System.nanoTime()
        setPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "initialTimestamp",
            initialTimestamp1
        )

        val randomizer = UniformRandomizer()
        val roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        processor.attitude.fromQuaternion(Quaternion(roll1, pitch1, yaw1))

        val roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val internalAttitude: Quaternion? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "internalAttitude"
        )
        requireNotNull(internalAttitude)
        internalAttitude.fromQuaternion(Quaternion(roll2, pitch2, yaw2))

        processor.reset()

        // check
        verify(exactly = 1) { timeIntervalEstimatorSpy.reset() }

        val initialTimestamp2: Long? = getPrivateProperty(
            BaseRelativeGyroscopeAttitudeProcessor::class,
            processor,
            "initialTimestamp"
        )
        requireNotNull(initialTimestamp2)
        assertEquals(0L, initialTimestamp2)

        assertEquals(Quaternion(), processor.attitude)
        assertEquals(Quaternion(), internalAttitude)
    }

    private companion object {
        const val INTERVAL_NANOS = 10_000_000 // 0,01 seconds

        const val INTERVAL_SECONDS = 0.01

        const val MIN_ANGLE_DEGREES = -45.0
        const val MAX_ANGLE_DEGREES = 45.0
    }
}