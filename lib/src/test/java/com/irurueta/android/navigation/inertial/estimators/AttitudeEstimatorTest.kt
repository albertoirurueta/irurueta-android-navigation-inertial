package com.irurueta.android.navigation.inertial.estimators

import android.content.Context
import android.hardware.Sensor
import android.location.Location
import android.os.SystemClock
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.navigation.inertial.callPrivateFunc
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.inertial.estimators.LevelingEstimator
import com.irurueta.navigation.inertial.estimators.LevelingEstimator2
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.MagneticFluxDensityConverter
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import java.util.*

@RunWith(RobolectricTestRunner::class)
class AttitudeEstimatorTest {

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context)

        // check
        assertSame(context, estimator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
            estimator.accelerometerSensorType
        )
        assertEquals(GyroscopeSensorCollector.SensorType.GYROSCOPE, estimator.gyroscopeSensorType)
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER,
            estimator.magnetometerSensorType
        )
        assertEquals(SensorDelay.UI, estimator.accelerometerSensorDelay)
        assertEquals(SensorDelay.UI, estimator.gyroscopeSensorDelay)
        assertEquals(SensorDelay.UI, estimator.magnetometerSensorDelay)
        assertNull(estimator.location)
        assertFalse(estimator.estimateImuLeveling)
        assertNull(estimator.worldMagneticModel)
        assertNotNull(estimator.timestamp)
        assertNull(estimator.attitudeAvailableListener)
        assertFalse(estimator.running)
        assertNull(estimator.nedPosition)
        assertNull(estimator.positionHorizontalAccuracyMeters)
        assertNull(estimator.positionVerticalAccuracyMeters)
        assertNull(estimator.accelerometerSensor)
        assertNull(estimator.gyroscopeSensor)
        assertNull(estimator.magnetometerSensor)
        assertFalse(estimator.accelerometerSensorAvailable)
        assertFalse(estimator.gyroscopeSensorAvailable)
        assertFalse(estimator.magnetometerSensorAvailable)
        assertFalse(estimator.isLevelingEnabled)
        assertFalse(estimator.isImprovedLevelingEnabled)
        assertFalse(estimator.isGeomagneticAttitudeEnabled)
        assertNull(estimator.type)
        assertFalse(estimator.isReady)
    }

    @Test
    fun constructor_whenAllProperties_setsExpectdValues() {
        val attitudeAvailableListener = mockk<AttitudeEstimator.OnAttitudeAvailableListener>()
        val timestamp = Date()
        val worldMagneticModel = WorldMagneticModel()
        val location = createLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(
            context,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            SensorDelay.GAME,
            SensorDelay.FASTEST,
            location,
            estimateImuLeveling = true,
            worldMagneticModel,
            timestamp,
            attitudeAvailableListener
        )

        // check
        assertSame(context, estimator.context)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            estimator.gyroscopeSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            estimator.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, estimator.accelerometerSensorDelay)
        assertEquals(SensorDelay.GAME, estimator.gyroscopeSensorDelay)
        assertEquals(SensorDelay.FASTEST, estimator.magnetometerSensorDelay)
        assertSame(location, estimator.location)
        assertTrue(estimator.estimateImuLeveling)
        assertSame(worldMagneticModel, estimator.worldMagneticModel)
        assertSame(timestamp, estimator.timestamp)
        assertSame(attitudeAvailableListener, estimator.attitudeAvailableListener)
        assertFalse(estimator.running)
        assertNotNull(estimator.nedPosition)
        assertEquals(location.toNEDPosition(), estimator.nedPosition)
        val positionHorizontalAccuracyMeters = estimator.positionHorizontalAccuracyMeters
        requireNotNull(positionHorizontalAccuracyMeters)
        assertEquals(location.accuracy, positionHorizontalAccuracyMeters, 0.0f)
        val positionVerticalAccuracyMeters = estimator.positionVerticalAccuracyMeters
        requireNotNull(positionVerticalAccuracyMeters)
        assertEquals(location.verticalAccuracyMeters, positionVerticalAccuracyMeters, 0.0f)
        assertNull(estimator.accelerometerSensor)
        assertNull(estimator.gyroscopeSensor)
        assertNull(estimator.magnetometerSensor)
        assertFalse(estimator.accelerometerSensorAvailable)
        assertFalse(estimator.gyroscopeSensorAvailable)
        assertFalse(estimator.magnetometerSensorAvailable)
        assertFalse(estimator.isLevelingEnabled)
        assertFalse(estimator.isImprovedLevelingEnabled)
        assertFalse(estimator.isGeomagneticAttitudeEnabled)
        assertNull(estimator.type)
        assertFalse(estimator.isReady)
    }

    @Test
    fun timestamp_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context)

        // check default value
        assertNotNull(estimator.timestamp)

        // set new value
        val timestamp = Date()
        estimator.timestamp = timestamp

        // check
        assertSame(timestamp, estimator.timestamp)
    }

    @Test
    fun attitudeAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context)

        // check default value
        assertNull(estimator.attitudeAvailableListener)

        // set new value
        val attitudeAvailableListener = mockk<AttitudeEstimator.OnAttitudeAvailableListener>()
        estimator.attitudeAvailableListener = attitudeAvailableListener

        // check
        assertSame(attitudeAvailableListener, estimator.attitudeAvailableListener)
    }

    @Test
    fun location_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context)

        // check default value
        assertNull(estimator.location)
        assertNull(estimator.nedPosition)
        assertNull(estimator.positionHorizontalAccuracyMeters)
        assertNull(estimator.positionVerticalAccuracyMeters)
        assertNull(estimator.type)

        // set new value
        val location = createLocation()
        estimator.location = location

        // check
        assertSame(location, estimator.location)
        val nedPosition = location.toNEDPosition()
        assertEquals(nedPosition, estimator.nedPosition)
        assertEquals(location.accuracy, estimator.positionHorizontalAccuracyMeters)
        assertEquals(location.verticalAccuracyMeters, estimator.positionVerticalAccuracyMeters)
        assertNull(estimator.type)
    }

    @Test(expected = IllegalStateException::class)
    fun location_whenRunningAndNotNullAndNewNullValue_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context)

        // set location
        val location = createLocation()
        estimator.location = location

        // check
        assertSame(location, estimator.location)
        assertFalse(estimator.running)

        // set as running
        estimator.setPrivateProperty("running", true)

        // set null location
        estimator.location = null
    }

    @Test
    fun worldMagneticModel_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context)

        // check default value
        assertNull(estimator.worldMagneticModel)
        assertNotNull(estimator.getPrivateProperty("attitudeEstimator"))

        // set new value
        val worldMagneticModel = WorldMagneticModel()
        estimator.worldMagneticModel = worldMagneticModel

        // check
        assertSame(worldMagneticModel, estimator.worldMagneticModel)
        assertNotNull(estimator.getPrivateProperty("attitudeEstimator"))

        // set as null
        estimator.worldMagneticModel = null

        // check
        assertNull(estimator.worldMagneticModel)
        assertNotNull(estimator.getPrivateProperty("attitudeEstimator"))
    }

    @Test(expected = IllegalStateException::class)
    fun worldMagneticModel_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context)

        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        val worldMagneticModel = WorldMagneticModel()
        estimator.worldMagneticModel = worldMagneticModel
    }

    @Test
    fun estimateImuLeveling_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context)

        // check default value
        assertFalse(estimator.estimateImuLeveling)
        assertNull(estimator.type)

        // set new value
        estimator.estimateImuLeveling = true

        // check
        assertTrue(estimator.estimateImuLeveling)
        assertNull(estimator.type)
    }

    @Test(expected = IllegalStateException::class)
    fun estimateImuLeveling_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context)

        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)
        assertFalse(estimator.estimateImuLeveling)

        estimator.estimateImuLeveling = true
    }

    @Test
    fun accelerometerSensor_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context)

        // check default value
        assertNull(estimator.accelerometerSensor)

        // set value
        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        val sensor = mockk<Sensor>()
        every { accelerometerSensorCollectorSpy.sensor }.returns(sensor)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        // check
        assertSame(sensor, estimator.accelerometerSensor)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.sensor }
    }

    @Test
    fun gyroscopeSensor_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context)

        // check default value
        assertNull(estimator.gyroscopeSensor)

        // set value
        val gyroscopeSensorCollector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        val sensor = mockk<Sensor>()
        every { gyroscopeSensorCollectorSpy.sensor }.returns(sensor)
        estimator.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        // check
        assertSame(sensor, estimator.gyroscopeSensor)
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.sensor }
    }

    @Test
    fun magnetometerSensor_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context)

        // check default value
        assertNull(estimator.magnetometerSensor)

        // set value
        val magnetometerSensorCollector: MagnetometerSensorCollector? =
            estimator.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        val sensor = mockk<Sensor>()
        every { magnetometerSensorCollectorSpy.sensor }.returns(sensor)
        estimator.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        // check
        assertSame(sensor, estimator.magnetometerSensor)
        verify(exactly = 1) { magnetometerSensorCollectorSpy.sensor }
    }

    @Test
    fun accelerometerSensorAvailable_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context)

        // check default value
        assertFalse(estimator.accelerometerSensorAvailable)

        // set value
        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.sensorAvailable }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        // check
        assertTrue(estimator.accelerometerSensorAvailable)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.sensorAvailable }
    }

    @Test
    fun gyroscopeSensorAvailable_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context)

        // check default value
        assertFalse(estimator.gyroscopeSensorAvailable)

        // set value
        val gyroscopeSensorCollector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        every { gyroscopeSensorCollectorSpy.sensorAvailable }.returns(true)
        estimator.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        // check
        assertTrue(estimator.gyroscopeSensorAvailable)
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.sensorAvailable }
    }

    @Test
    fun magnetometerSensorAvailable_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context)

        // check default value
        assertFalse(estimator.magnetometerSensorAvailable)

        // set value
        val magnetometerSensorCollector: MagnetometerSensorCollector? =
            estimator.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        every { magnetometerSensorCollectorSpy.sensorAvailable }.returns(true)
        estimator.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        // check
        assertTrue(estimator.magnetometerSensorAvailable)
        verify(exactly = 1) { magnetometerSensorCollectorSpy.sensorAvailable }
    }

    @Test
    fun isLevelingEnabled_whenNotEstimateImuLeveling_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context)

        // check
        assertFalse(estimator.isLevelingEnabled)
        assertFalse(estimator.estimateImuLeveling)
        assertNull(estimator.type)
    }

    @Test
    fun isLevelingEnabled_whenEstimateImuLevelingAndAccelerometerSensorNotAvailable_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context, estimateImuLeveling = true)

        // check
        assertFalse(estimator.isLevelingEnabled)
        assertTrue(estimator.estimateImuLeveling)
        assertFalse(estimator.accelerometerSensorAvailable)
        assertNull(estimator.type)
    }

    @Test
    fun isLevelingEnabled_whenEstimateImuLevelingAccelerometerSensorAvailableAndGyroscopeSensorNotAvailable_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context, estimateImuLeveling = true)

        // set accelerometerSensorAvailable value
        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.sensorAvailable }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        // check
        assertFalse(estimator.isLevelingEnabled)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.sensorAvailable }
        assertTrue(estimator.estimateImuLeveling)
        assertTrue(estimator.accelerometerSensorAvailable)
        assertFalse(estimator.gyroscopeSensorAvailable)
        assertNull(estimator.type)
    }

    @Test
    fun isLevelingEnabled_whenEstimateImuLevelingAccelerometerSensorAvailableAndGyroscopeSensorAvailable_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context, estimateImuLeveling = true)

        // set accelerometerSensorAvailable value
        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.sensorAvailable }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        // set gyroscopeSensorAvailable value
        val gyroscopeSensorCollector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        every { gyroscopeSensorCollectorSpy.sensorAvailable }.returns(true)
        estimator.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        // check
        assertTrue(estimator.isLevelingEnabled)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.sensorAvailable }
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.sensorAvailable }
        assertTrue(estimator.estimateImuLeveling)
        assertTrue(estimator.accelerometerSensorAvailable)
        assertTrue(estimator.gyroscopeSensorAvailable)
        assertNull(estimator.type)

        callPrivateFunc(AttitudeEstimator::class, estimator, "computeType")
        assertEquals(AttitudeEstimator.AttitudeEstimatorType.LEVELING, estimator.type)
    }

    @Test
    fun isImprovedLevelingEnabled_whenNoLocation_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context)

        // check
        assertFalse(estimator.isImprovedLevelingEnabled)
        assertNull(estimator.location)
        assertNull(estimator.type)
    }

    @Test
    fun isImprovedLevelingEnabled_whenLocationAndNoEstimateImuLeveling_returnsFalse() {
        val location = createLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context, location = location)

        // check
        assertFalse(estimator.isImprovedLevelingEnabled)
        assertSame(location, estimator.location)
        assertFalse(estimator.estimateImuLeveling)
        assertNull(estimator.type)
    }

    @Test
    fun isImprovedLevelingEnabled_whenLocationEstimateImuLevelingAndAccelerometerSensorNotAvailable_returnsFalse() {
        val location = createLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context, location = location, estimateImuLeveling = true)

        // check
        assertFalse(estimator.isImprovedLevelingEnabled)
        assertSame(location, estimator.location)
        assertTrue(estimator.estimateImuLeveling)
        assertFalse(estimator.accelerometerSensorAvailable)
        assertNull(estimator.type)
    }

    @Test
    fun isImprovedLevelingEnabled_whenLocationEstimateImuLevelingAccelerometerSensorAvailableAndGyroscopeSensorNotAvailable_returnsFalse() {
        val location = createLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context, location = location, estimateImuLeveling = true)

        // set accelerometerSensorAvailable value
        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.sensorAvailable }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        // check
        assertFalse(estimator.isImprovedLevelingEnabled)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.sensorAvailable }
        assertSame(location, estimator.location)
        assertTrue(estimator.estimateImuLeveling)
        assertTrue(estimator.accelerometerSensorAvailable)
        assertFalse(estimator.gyroscopeSensorAvailable)
        assertNull(estimator.type)
    }

    @Test
    fun isImprovedLevelingEnabled_whenLocationEstimateImuLevelingAccelerometerSensorAvailableAndGyroscopeSensorAvailable_returnsTrue() {
        val location = createLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context, location = location, estimateImuLeveling = true)

        // set accelerometerSensorAvailable value
        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.sensorAvailable }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        // set gyroscopeSensorAvailable value
        val gyroscopeSensorCollector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        every { gyroscopeSensorCollectorSpy.sensorAvailable }.returns(true)
        estimator.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        // check
        assertTrue(estimator.isImprovedLevelingEnabled)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.sensorAvailable }
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.sensorAvailable }
        assertSame(location, estimator.location)
        assertTrue(estimator.estimateImuLeveling)
        assertTrue(estimator.accelerometerSensorAvailable)
        assertTrue(estimator.gyroscopeSensorAvailable)
        assertNull(estimator.type)

        callPrivateFunc(AttitudeEstimator::class, estimator, "computeType")
        assertEquals(AttitudeEstimator.AttitudeEstimatorType.IMPROVED_LEVELING, estimator.type)
    }

    @Test
    fun isGeomagneticAttitudeEnabled_whenNoLocation_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context)

        // check
        assertFalse(estimator.isGeomagneticAttitudeEnabled)
        assertNull(estimator.location)
        assertNull(estimator.type)
    }

    @Test
    fun isGeomagneticAttitudeEnabled_whenLocationAndEstimateImuLeveling_returnsFalse() {
        val location = createLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context, location = location, estimateImuLeveling = true)

        assertFalse(estimator.isGeomagneticAttitudeEnabled)
        assertSame(location, estimator.location)
        assertTrue(estimator.estimateImuLeveling)
        assertNull(estimator.type)
    }

    @Test
    fun isGeomagneticAttitudeEnabled_whenLocationNotEstimateImuLevelingAndAccelerometerSensorNotAvailable_returnsFalse() {
        val location = createLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context, location = location, estimateImuLeveling = false)

        assertFalse(estimator.isGeomagneticAttitudeEnabled)
        assertSame(location, estimator.location)
        assertFalse(estimator.estimateImuLeveling)
        assertFalse(estimator.accelerometerSensorAvailable)
        assertNull(estimator.type)
    }

    @Test
    fun isGeomagneticAttitudeEnabled_whenLocationNotEstimateImuLevelingAccelerometerSensorAvailableAndMagnetometerSensorNotAvailable_returnsFalse() {
        val location = createLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context, location = location, estimateImuLeveling = false)

        // set accelerometerSensorAvailable value
        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.sensorAvailable }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        assertFalse(estimator.isGeomagneticAttitudeEnabled)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.sensorAvailable }
        assertSame(location, estimator.location)
        assertFalse(estimator.estimateImuLeveling)
        assertTrue(estimator.accelerometerSensorAvailable)
        assertFalse(estimator.magnetometerSensorAvailable)
        assertNull(estimator.type)
    }

    @Test
    fun isGeomagneticAttitudeEnabled_whenLocationNotEstimateImuLevelingAccelerometerSensorAvailableAndMagnetometerSensorAvailable_returnsTue() {
        val location = createLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context, location = location, estimateImuLeveling = false)

        // set accelerometerSensorAvailable value
        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.sensorAvailable }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        // set magnetometerSensorAvailable value
        val magnetometerSensorCollector: MagnetometerSensorCollector? =
            estimator.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        every { magnetometerSensorCollectorSpy.sensorAvailable }.returns(true)
        estimator.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        assertTrue(estimator.isGeomagneticAttitudeEnabled)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.sensorAvailable }
        verify(exactly = 1) { magnetometerSensorCollectorSpy.sensorAvailable }
        assertSame(location, estimator.location)
        assertFalse(estimator.estimateImuLeveling)
        assertTrue(estimator.accelerometerSensorAvailable)
        assertTrue(estimator.magnetometerSensorAvailable)
        assertNull(estimator.type)

        callPrivateFunc(AttitudeEstimator::class, estimator, "computeType")
        assertEquals(AttitudeEstimator.AttitudeEstimatorType.GEOMAGNETIC, estimator.type)
    }

    @Test
    fun typeAndIsReady_whenGeomagnetic_returnsExpectedValue() {
        val location = createLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context, location = location)

        // set accelerometerSensorAvailable value
        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.sensorAvailable }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        // set magnetometerSensorAvailable value
        val magnetometerSensorCollector: MagnetometerSensorCollector? =
            estimator.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        every { magnetometerSensorCollectorSpy.sensorAvailable }.returns(true)
        estimator.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        assertTrue(estimator.isGeomagneticAttitudeEnabled)
        assertNull(estimator.type)

        callPrivateFunc(AttitudeEstimator::class, estimator, "computeType")
        assertEquals(AttitudeEstimator.AttitudeEstimatorType.GEOMAGNETIC, estimator.type)

        assertTrue(estimator.isReady)
    }

    @Test
    fun typeAndIsReady_whenImprovedLeveling_returnsExpectedValue() {
        val location = createLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context, location = location, estimateImuLeveling = true)

        // set accelerometerSensorAvailable value
        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.sensorAvailable }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        // set gyroscopeSensorAvailable value
        val gyroscopeSensorCollector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        every { gyroscopeSensorCollectorSpy.sensorAvailable }.returns(true)
        estimator.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        assertTrue(estimator.isImprovedLevelingEnabled)
        assertNull(estimator.type)

        callPrivateFunc(AttitudeEstimator::class, estimator, "computeType")
        assertEquals(AttitudeEstimator.AttitudeEstimatorType.IMPROVED_LEVELING, estimator.type)

        assertTrue(estimator.isReady)
    }

    @Test
    fun typeAndIsReady_whenLeveling_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context, estimateImuLeveling = true)

        // set accelerometerSensorAvailable value
        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.sensorAvailable }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        // set gyroscopeSensorAvailable value
        val gyroscopeSensorCollector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        every { gyroscopeSensorCollectorSpy.sensorAvailable }.returns(true)
        estimator.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        assertTrue(estimator.isLevelingEnabled)
        assertNull(estimator.type)

        callPrivateFunc(AttitudeEstimator::class, estimator, "computeType")
        assertEquals(AttitudeEstimator.AttitudeEstimatorType.LEVELING, estimator.type)

        assertTrue(estimator.isReady)
    }

    @Test
    fun typeAndIsReady_whenUnknown_returnsNull() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context, estimateImuLeveling = true)

        assertNull(estimator.type)

        callPrivateFunc(AttitudeEstimator::class, estimator, "computeType")
        assertNull(estimator.type)

        assertFalse(estimator.isReady)
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenNotReady_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context)

        assertFalse(estimator.isReady)
        estimator.start()
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenReadyAndAlreadyRunning_throwsIllegalStateException() {
        val location = createLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context, location = location)

        // set accelerometerSensorAvailable value
        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.sensorAvailable }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        // set magnetometerSensorAvailable value
        val magnetometerSensorCollector: MagnetometerSensorCollector? =
            estimator.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        every { magnetometerSensorCollectorSpy.sensorAvailable }.returns(true)
        estimator.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        assertTrue(estimator.isReady)

        // set as running
        estimator.setPrivateProperty("running", true)

        estimator.start()
    }

    @Test
    fun start_whenReadyNotRunningGeomagneticAndSensorsStart_startsAccelerometerAndMagnetometerAndReturnsTrue() {
        val location = createLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context, location = location)

        // set accelerometerSensorAvailable value
        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.sensorAvailable }.returns(true)
        every { accelerometerSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        // set magnetometerSensorAvailable value
        val magnetometerSensorCollector: MagnetometerSensorCollector? =
            estimator.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        every { magnetometerSensorCollectorSpy.sensorAvailable }.returns(true)
        every { magnetometerSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        assertTrue(estimator.isReady)

        assertTrue(estimator.start())
        verify(exactly = 1) { accelerometerSensorCollectorSpy.start() }
        verify(exactly = 1) { magnetometerSensorCollectorSpy.start() }
        assertTrue(estimator.running)

        val gyroscopeSampleAvailable: Boolean? =
            estimator.getPrivateProperty("gyroscopeSampleAvailable")
        requireNotNull(gyroscopeSampleAvailable)
        assertTrue(gyroscopeSampleAvailable)
    }

    @Test
    fun start_whenReadyNotRunningGeomagneticAndAccelerometerStartFails_stopsSensorsAndReturnsFalse() {
        val location = createLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context, location = location)

        // set accelerometerSensorAvailable value
        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.sensorAvailable }.returns(true)
        every { accelerometerSensorCollectorSpy.start() }.returns(false)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        // set magnetometerSensorAvailable value
        val magnetometerSensorCollector: MagnetometerSensorCollector? =
            estimator.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        every { magnetometerSensorCollectorSpy.sensorAvailable }.returns(true)
        every { magnetometerSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        assertTrue(estimator.isReady)

        assertFalse(estimator.start())
        verify(exactly = 1) { accelerometerSensorCollectorSpy.start() }
        verify(exactly = 0) { magnetometerSensorCollectorSpy.start() }
        verify(exactly = 1) { accelerometerSensorCollectorSpy.stop() }
        verify(exactly = 1) { magnetometerSensorCollectorSpy.stop() }
        assertFalse(estimator.running)

        val gyroscopeSampleAvailable: Boolean? =
            estimator.getPrivateProperty("gyroscopeSampleAvailable")
        requireNotNull(gyroscopeSampleAvailable)
        assertFalse(gyroscopeSampleAvailable)
    }

    @Test
    fun start_whenReadyNotRunningGeomagneticAndGyroscopeStartFails_stopsSensorsAndReturnsFalse() {
        val location = createLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context, location = location)

        // set accelerometerSensorAvailable value
        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.sensorAvailable }.returns(true)
        every { accelerometerSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        // set magnetometerSensorAvailable value
        val magnetometerSensorCollector: MagnetometerSensorCollector? =
            estimator.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        every { magnetometerSensorCollectorSpy.sensorAvailable }.returns(true)
        every { magnetometerSensorCollectorSpy.start() }.returns(false)
        estimator.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        assertTrue(estimator.isReady)

        assertFalse(estimator.start())
        verify(exactly = 1) { accelerometerSensorCollectorSpy.start() }
        verify(exactly = 1) { magnetometerSensorCollectorSpy.start() }
        verify(exactly = 1) { accelerometerSensorCollectorSpy.stop() }
        verify(exactly = 1) { magnetometerSensorCollectorSpy.stop() }
        assertFalse(estimator.running)

        val gyroscopeSampleAvailable: Boolean? =
            estimator.getPrivateProperty("gyroscopeSampleAvailable")
        requireNotNull(gyroscopeSampleAvailable)
        assertFalse(gyroscopeSampleAvailable)
    }

    @Test
    fun start_whenReadyNotRunningLevelingAndSensorsStart_startsAccelerometerAndGyroscopeAndReturnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context, estimateImuLeveling = true)

        // set accelerometerSensorAvailable value
        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.sensorAvailable }.returns(true)
        every { accelerometerSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        // set gyroscopeSensorAvailable value
        val gyroscopeSensorCollector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        every { gyroscopeSensorCollectorSpy.sensorAvailable }.returns(true)
        every { gyroscopeSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        assertTrue(estimator.isReady)

        assertTrue(estimator.start())
        verify(exactly = 1) { accelerometerSensorCollectorSpy.start() }
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.start() }
        assertTrue(estimator.running)

        val magnetometerSampleAvailable: Boolean? =
            estimator.getPrivateProperty("magnetometerSampleAvailable")
        requireNotNull(magnetometerSampleAvailable)
        assertTrue(magnetometerSampleAvailable)
    }

    @Test
    fun start_whenReadyNotRunningLevelingAndAccelerometerStartFails_stopsSensorsAndReturnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context, estimateImuLeveling = true)

        // set accelerometerSensorAvailable value
        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.sensorAvailable }.returns(true)
        every { accelerometerSensorCollectorSpy.start() }.returns(false)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        // set gyroscopeSensorAvailable value
        val gyroscopeSensorCollector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        every { gyroscopeSensorCollectorSpy.sensorAvailable }.returns(true)
        every { gyroscopeSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        assertTrue(estimator.isReady)

        assertFalse(estimator.start())
        verify(exactly = 1) { accelerometerSensorCollectorSpy.start() }
        verify(exactly = 0) { gyroscopeSensorCollectorSpy.start() }
        verify(exactly = 1) { accelerometerSensorCollectorSpy.stop() }
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.stop() }
        assertFalse(estimator.running)

        val magnetometerSampleAvailable: Boolean? =
            estimator.getPrivateProperty("magnetometerSampleAvailable")
        requireNotNull(magnetometerSampleAvailable)
        assertFalse(magnetometerSampleAvailable)
    }

    @Test
    fun start_whenReadyNotRunningLevelingAndGyroscopeStartFails_stopsSensorsAndReturnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context, estimateImuLeveling = true)

        // set accelerometerSensorAvailable value
        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.sensorAvailable }.returns(true)
        every { accelerometerSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        // set gyroscopeSensorAvailable value
        val gyroscopeSensorCollector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        every { gyroscopeSensorCollectorSpy.sensorAvailable }.returns(true)
        every { gyroscopeSensorCollectorSpy.start() }.returns(false)
        estimator.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        assertTrue(estimator.isReady)

        assertFalse(estimator.start())
        verify(exactly = 1) { accelerometerSensorCollectorSpy.start() }
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.start() }
        verify(exactly = 1) { accelerometerSensorCollectorSpy.stop() }
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.stop() }
        assertFalse(estimator.running)

        val magnetometerSampleAvailable: Boolean? =
            estimator.getPrivateProperty("magnetometerSampleAvailable")
        requireNotNull(magnetometerSampleAvailable)
        assertFalse(magnetometerSampleAvailable)
    }

    @Test
    fun stop_stopsAllCollectorsAndResets() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(context)

        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        val gyroscopeSensorCollector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        estimator.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        val magnetometerSensorCollector: MagnetometerSensorCollector? =
            estimator.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        estimator.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        estimator.setPrivateProperty("accelerometerSampleAvailable", true)
        estimator.setPrivateProperty("gyroscopeSampleAvailable", true)
        estimator.setPrivateProperty("magnetometerSampleAvailable", true)
        estimator.setPrivateProperty("running", true)

        // check
        val accelerometerSampleAvailable1: Boolean? =
            estimator.getPrivateProperty("accelerometerSampleAvailable")
        requireNotNull(accelerometerSampleAvailable1)
        assertTrue(accelerometerSampleAvailable1)
        val gyroscopeSampleAvailable1: Boolean? =
            estimator.getPrivateProperty("gyroscopeSampleAvailable")
        requireNotNull(gyroscopeSampleAvailable1)
        assertTrue(gyroscopeSampleAvailable1)
        val magnetometerSampleAvailable1: Boolean? =
            estimator.getPrivateProperty("magnetometerSampleAvailable")
        requireNotNull(magnetometerSampleAvailable1)
        assertTrue(magnetometerSampleAvailable1)
        assertTrue(estimator.running)

        estimator.stop()

        // check
        verify(exactly = 1) { magnetometerSensorCollectorSpy.stop() }
        verify(exactly = 1) { accelerometerSensorCollectorSpy.stop() }
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.stop() }

        // check
        val accelerometerSampleAvailable2: Boolean? =
            estimator.getPrivateProperty("accelerometerSampleAvailable")
        requireNotNull(accelerometerSampleAvailable2)
        assertFalse(accelerometerSampleAvailable2)
        val gyroscopeSampleAvailable2: Boolean? =
            estimator.getPrivateProperty("gyroscopeSampleAvailable")
        requireNotNull(gyroscopeSampleAvailable2)
        assertFalse(gyroscopeSampleAvailable2)
        val magnetometerSampleAvailable2: Boolean? =
            estimator.getPrivateProperty("magnetometerSampleAvailable")
        requireNotNull(magnetometerSampleAvailable2)
        assertFalse(magnetometerSampleAvailable2)
        assertFalse(estimator.running)
    }

    @Test
    fun accelerometerSensorCollector_whenNotAllSamplesAvailable_storesMeasurementAndNotNotifies() {
        val attitudeAvailableListener = mockk<AttitudeEstimator.OnAttitudeAvailableListener>()
        val location = createLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(
            context,
            location = location,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)

        val accelerometerMeasurementListener = accelerometerSensorCollector.measurementListener
        requireNotNull(accelerometerMeasurementListener)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
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

        // check
        assertEquals(ax, estimator.getPrivateProperty("ax"))
        assertEquals(ay, estimator.getPrivateProperty("ay"))
        assertEquals(az, estimator.getPrivateProperty("az"))
        val accelerometerSampleAvailable: Boolean? =
            estimator.getPrivateProperty("accelerometerSampleAvailable")
        requireNotNull(accelerometerSampleAvailable)
        assertTrue(accelerometerSampleAvailable)
        val gyroscopeSampleAvailable: Boolean? =
            estimator.getPrivateProperty("gyroscopeSampleAvailable")
        requireNotNull(gyroscopeSampleAvailable)
        assertFalse(gyroscopeSampleAvailable)
        val magnetometerSampleAvailable: Boolean? =
            estimator.getPrivateProperty("magnetometerSampleAvailable")
        requireNotNull(magnetometerSampleAvailable)
        assertFalse(magnetometerSampleAvailable)
        verify { attitudeAvailableListener wasNot Called }
    }

    @Test
    fun gyroscopeSensorCollector_whenNotAllSamplesAvailable_storesMeasurementAndNotNotifies() {
        val attitudeAvailableListener = mockk<AttitudeEstimator.OnAttitudeAvailableListener>()
        val location = createLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(
            context,
            location = location,
            attitudeAvailableListener = attitudeAvailableListener,
            estimateImuLeveling = true
        )

        val gyroscopeSensorCollector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)

        val gyroscopeMeasurementListener = gyroscopeSensorCollector.measurementListener
        requireNotNull(gyroscopeMeasurementListener)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        gyroscopeMeasurementListener.onMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH
        )

        // check
        assertEquals(wx, estimator.getPrivateProperty("wx"))
        assertEquals(wy, estimator.getPrivateProperty("wy"))
        assertEquals(wz, estimator.getPrivateProperty("wz"))
        val accelerometerSampleAvailable: Boolean? =
            estimator.getPrivateProperty("accelerometerSampleAvailable")
        requireNotNull(accelerometerSampleAvailable)
        assertFalse(accelerometerSampleAvailable)
        val gyroscopeSampleAvailable: Boolean? =
            estimator.getPrivateProperty("gyroscopeSampleAvailable")
        requireNotNull(gyroscopeSampleAvailable)
        assertTrue(gyroscopeSampleAvailable)
        val magnetometerSampleAvailable: Boolean? =
            estimator.getPrivateProperty("magnetometerSampleAvailable")
        requireNotNull(magnetometerSampleAvailable)
        assertFalse(magnetometerSampleAvailable)
        verify { attitudeAvailableListener wasNot Called }
    }

    @Test
    fun magnetometerSensorCollector_whenNotAllSamplesAvailable_storesMeasurementAndNotNotifies() {
        val attitudeAvailableListener = mockk<AttitudeEstimator.OnAttitudeAvailableListener>()
        val location = createLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(
            context,
            location = location,
            attitudeAvailableListener = attitudeAvailableListener,
            estimateImuLeveling = true
        )

        val magnetometerSensorCollector: MagnetometerSensorCollector? =
            estimator.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)

        val magnetometerMeasurementListener = magnetometerSensorCollector.measurementListener
        requireNotNull(magnetometerMeasurementListener)

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        magnetometerMeasurementListener.onMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            SensorAccuracy.HIGH
        )

        // check
        assertEquals(bx, estimator.getPrivateProperty("bx"))
        assertEquals(by, estimator.getPrivateProperty("by"))
        assertEquals(bz, estimator.getPrivateProperty("bz"))
        val accelerometerSampleAvailable: Boolean? =
            estimator.getPrivateProperty("accelerometerSampleAvailable")
        requireNotNull(accelerometerSampleAvailable)
        assertFalse(accelerometerSampleAvailable)
        val gyroscopeSampleAvailable: Boolean? =
            estimator.getPrivateProperty("gyroscopeSampleAvailable")
        requireNotNull(gyroscopeSampleAvailable)
        assertFalse(gyroscopeSampleAvailable)
        val magnetometerSampleAvailable: Boolean? =
            estimator.getPrivateProperty("magnetometerSampleAvailable")
        requireNotNull(magnetometerSampleAvailable)
        assertTrue(magnetometerSampleAvailable)
        verify { attitudeAvailableListener wasNot Called }
    }

    @Test
    fun processSamples_whenGeomagnetic_processesGeomagneticAttitudeAndNotifies() {
        val attitudeAvailableListener =
            mockk<AttitudeEstimator.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val location = createLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(
            context,
            location = location,
            attitudeAvailableListener = attitudeAvailableListener
        )

        // set accelerometerSensorAvailable value
        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.sensorAvailable }.returns(true)
        every { accelerometerSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        // set magnetometerSensorAvailable value
        val magnetometerSensorCollector: MagnetometerSensorCollector? =
            estimator.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        every { magnetometerSensorCollectorSpy.sensorAvailable }.returns(true)
        every { magnetometerSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        callPrivateFunc(AttitudeEstimator::class, estimator, "computeType")
        assertEquals(AttitudeEstimator.AttitudeEstimatorType.GEOMAGNETIC, estimator.type)

        assertTrue(estimator.isReady)
        val accelerometerSampleAvailable1: Boolean? =
            estimator.getPrivateProperty("accelerometerSampleAvailable")
        requireNotNull(accelerometerSampleAvailable1)
        assertFalse(accelerometerSampleAvailable1)
        val gyroscopeSampleAvailable1: Boolean? =
            estimator.getPrivateProperty("gyroscopeSampleAvailable")
        requireNotNull(gyroscopeSampleAvailable1)
        assertFalse(gyroscopeSampleAvailable1)
        val magnetometerSampleAvailable1: Boolean? =
            estimator.getPrivateProperty("magnetometerSampleAvailable")
        requireNotNull(magnetometerSampleAvailable1)
        assertFalse(magnetometerSampleAvailable1)

        val internalAttitudeEstimator: com.irurueta.navigation.inertial.estimators.AttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(internalAttitudeEstimator)
        val internalAttitudeEstimatorSpy = spyk(internalAttitudeEstimator)
        estimator.setPrivateProperty("attitudeEstimator", internalAttitudeEstimatorSpy)

        assertTrue(estimator.start())

        // collect accelerometer measurement
        val accelerometerSampleAvailable2: Boolean? =
            estimator.getPrivateProperty("accelerometerSampleAvailable")
        requireNotNull(accelerometerSampleAvailable2)
        assertFalse(accelerometerSampleAvailable2)
        val gyroscopeSampleAvailable2: Boolean? =
            estimator.getPrivateProperty("gyroscopeSampleAvailable")
        requireNotNull(gyroscopeSampleAvailable2)
        assertTrue(gyroscopeSampleAvailable2)
        val magnetometerSampleAvailable2: Boolean? =
            estimator.getPrivateProperty("magnetometerSampleAvailable")
        requireNotNull(magnetometerSampleAvailable2)
        assertFalse(magnetometerSampleAvailable2)

        val accelerometerMeasurementListener = accelerometerSensorCollector.measurementListener
        requireNotNull(accelerometerMeasurementListener)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val biasX = randomizer.nextFloat()
        val biasY = randomizer.nextFloat()
        val biasZ = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        accelerometerMeasurementListener.onMeasurement(
            ax,
            ay,
            az,
            biasX,
            biasY,
            biasZ,
            timestamp,
            SensorAccuracy.HIGH
        )

        // check
        assertEquals(ax, estimator.getPrivateProperty("ax"))
        assertEquals(ay, estimator.getPrivateProperty("ay"))
        assertEquals(az, estimator.getPrivateProperty("az"))
        val accelerometerSampleAvailable3: Boolean? =
            estimator.getPrivateProperty("accelerometerSampleAvailable")
        requireNotNull(accelerometerSampleAvailable3)
        assertTrue(accelerometerSampleAvailable3)
        val gyroscopeSampleAvailable3: Boolean? =
            estimator.getPrivateProperty("gyroscopeSampleAvailable")
        requireNotNull(gyroscopeSampleAvailable3)
        assertTrue(gyroscopeSampleAvailable3)
        val magnetometerSampleAvailable3: Boolean? =
            estimator.getPrivateProperty("magnetometerSampleAvailable")
        requireNotNull(magnetometerSampleAvailable3)
        assertFalse(magnetometerSampleAvailable3)

        val magnetometerMeasurementListener = magnetometerSensorCollector.measurementListener
        requireNotNull(magnetometerMeasurementListener)

        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        magnetometerMeasurementListener.onMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            SensorAccuracy.HIGH
        )

        // check
        assertEquals(ax, estimator.getPrivateProperty("ax"))
        assertEquals(ay, estimator.getPrivateProperty("ay"))
        assertEquals(az, estimator.getPrivateProperty("az"))
        assertEquals(bx, estimator.getPrivateProperty("bx"))
        assertEquals(by, estimator.getPrivateProperty("by"))
        assertEquals(bz, estimator.getPrivateProperty("bz"))
        val accelerometerSampleAvailable4: Boolean? =
            estimator.getPrivateProperty("accelerometerSampleAvailable")
        requireNotNull(accelerometerSampleAvailable4)
        assertFalse(accelerometerSampleAvailable4)
        val gyroscopeSampleAvailable4: Boolean? =
            estimator.getPrivateProperty("gyroscopeSampleAvailable")
        requireNotNull(gyroscopeSampleAvailable4)
        assertFalse(gyroscopeSampleAvailable4)
        val magnetometerSampleAvailable4: Boolean? =
            estimator.getPrivateProperty("magnetometerSampleAvailable")
        requireNotNull(magnetometerSampleAvailable4)
        assertFalse(magnetometerSampleAvailable4)

        val nedPosition = estimator.nedPosition
        requireNotNull(nedPosition)
        val bxTesla = MagneticFluxDensityConverter.microTeslaToTesla(bx.toDouble())
        val byTesla = MagneticFluxDensityConverter.microTeslaToTesla(by.toDouble())
        val bzTesla = MagneticFluxDensityConverter.microTeslaToTesla(bz.toDouble())
        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        requireNotNull(coordinateTransformation)
        verify(exactly = 1) {
            internalAttitudeEstimatorSpy.getAttitude(
                nedPosition.latitude,
                nedPosition.longitude,
                nedPosition.height,
                estimator.timestamp,
                ax.toDouble(),
                ay.toDouble(),
                az.toDouble(),
                bxTesla,
                byTesla,
                bzTesla,
                coordinateTransformation
            )
        }
        val attitude: Quaternion? = estimator.getPrivateProperty("attitude")
        requireNotNull(attitude)
        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                attitude,
                coordinateTransformation,
                AttitudeEstimator.AttitudeEstimatorType.GEOMAGNETIC
            )
        }
    }

    @Test
    fun processSamples_whenImprovedLeveling_processesImprovedLevelingAndNotifies() {
        val attitudeAvailableListener =
            mockk<AttitudeEstimator.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val location = createLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(
            context,
            location = location,
            attitudeAvailableListener = attitudeAvailableListener,
            estimateImuLeveling = true
        )

        // set accelerometerSensorAvailable value
        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.sensorAvailable }.returns(true)
        every { accelerometerSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        // set gyroscopeSensorAvailable value
        val gyroscopeSensorCollector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        every { gyroscopeSensorCollectorSpy.sensorAvailable }.returns(true)
        every { gyroscopeSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        callPrivateFunc(AttitudeEstimator::class, estimator, "computeType")
        assertEquals(AttitudeEstimator.AttitudeEstimatorType.IMPROVED_LEVELING, estimator.type)

        mockkStatic(LevelingEstimator2::class)

        assertTrue(estimator.isReady)
        val accelerometerSampleAvailable1: Boolean? =
            estimator.getPrivateProperty("accelerometerSampleAvailable")
        requireNotNull(accelerometerSampleAvailable1)
        assertFalse(accelerometerSampleAvailable1)
        val gyroscopeSampleAvailable1: Boolean? =
            estimator.getPrivateProperty("gyroscopeSampleAvailable")
        requireNotNull(gyroscopeSampleAvailable1)
        assertFalse(gyroscopeSampleAvailable1)
        val magnetometerSampleAvailable1: Boolean? =
            estimator.getPrivateProperty("magnetometerSampleAvailable")
        requireNotNull(magnetometerSampleAvailable1)
        assertFalse(magnetometerSampleAvailable1)

        assertTrue(estimator.start())

        // collect accelerometer measurement
        val accelerometerSampleAvailable2: Boolean? =
            estimator.getPrivateProperty("accelerometerSampleAvailable")
        requireNotNull(accelerometerSampleAvailable2)
        assertFalse(accelerometerSampleAvailable2)
        val gyroscopeSampleAvailable2: Boolean? =
            estimator.getPrivateProperty("gyroscopeSampleAvailable")
        requireNotNull(gyroscopeSampleAvailable2)
        assertFalse(gyroscopeSampleAvailable2)
        val magnetometerSampleAvailable2: Boolean? =
            estimator.getPrivateProperty("magnetometerSampleAvailable")
        requireNotNull(magnetometerSampleAvailable2)
        assertTrue(magnetometerSampleAvailable2)

        val accelerometerMeasurementListener = accelerometerSensorCollector.measurementListener
        requireNotNull(accelerometerMeasurementListener)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val biasX = randomizer.nextFloat()
        val biasY = randomizer.nextFloat()
        val biasZ = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        accelerometerMeasurementListener.onMeasurement(
            ax,
            ay,
            az,
            biasX,
            biasY,
            biasZ,
            timestamp,
            SensorAccuracy.HIGH
        )

        // check
        assertEquals(ax, estimator.getPrivateProperty("ax"))
        assertEquals(ay, estimator.getPrivateProperty("ay"))
        assertEquals(az, estimator.getPrivateProperty("az"))
        val accelerometerSampleAvailable3: Boolean? =
            estimator.getPrivateProperty("accelerometerSampleAvailable")
        requireNotNull(accelerometerSampleAvailable3)
        assertTrue(accelerometerSampleAvailable3)
        val gyroscopeSampleAvailable3: Boolean? =
            estimator.getPrivateProperty("gyroscopeSampleAvailable")
        requireNotNull(gyroscopeSampleAvailable3)
        assertFalse(gyroscopeSampleAvailable3)
        val magnetometerSampleAvailable3: Boolean? =
            estimator.getPrivateProperty("magnetometerSampleAvailable")
        requireNotNull(magnetometerSampleAvailable3)
        assertTrue(magnetometerSampleAvailable3)

        val gyroscopeMeasurementListener = gyroscopeSensorCollector.measurementListener
        requireNotNull(gyroscopeMeasurementListener)

        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        gyroscopeMeasurementListener.onMeasurement(
            wx,
            wy,
            wz,
            biasX,
            biasY,
            biasZ,
            timestamp,
            SensorAccuracy.HIGH
        )

        // check
        assertEquals(ax, estimator.getPrivateProperty("ax"))
        assertEquals(ay, estimator.getPrivateProperty("ay"))
        assertEquals(az, estimator.getPrivateProperty("az"))
        assertEquals(wx, estimator.getPrivateProperty("wx"))
        assertEquals(wy, estimator.getPrivateProperty("wy"))
        assertEquals(wz, estimator.getPrivateProperty("wz"))
        val accelerometerSampleAvailable4: Boolean? =
            estimator.getPrivateProperty("accelerometerSampleAvailable")
        requireNotNull(accelerometerSampleAvailable4)
        assertFalse(accelerometerSampleAvailable4)
        val gyroscopeSampleAvailable4: Boolean? =
            estimator.getPrivateProperty("gyroscopeSampleAvailable")
        requireNotNull(gyroscopeSampleAvailable4)
        assertFalse(gyroscopeSampleAvailable4)
        val magnetometerSampleAvailable4: Boolean? =
            estimator.getPrivateProperty("magnetometerSampleAvailable")
        requireNotNull(magnetometerSampleAvailable4)
        assertFalse(magnetometerSampleAvailable4)

        val nedPosition = estimator.nedPosition
        requireNotNull(nedPosition)
        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        requireNotNull(coordinateTransformation)
        verify(exactly = 1) {
            LevelingEstimator2.getAttitude(
                nedPosition,
                ax.toDouble(),
                ay.toDouble(),
                az.toDouble(),
                wx.toDouble(),
                wy.toDouble(),
                wz.toDouble(),
                coordinateTransformation
            )
        }
        val attitude: Quaternion? = estimator.getPrivateProperty("attitude")
        requireNotNull(attitude)
        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                attitude,
                coordinateTransformation,
                AttitudeEstimator.AttitudeEstimatorType.IMPROVED_LEVELING
            )
        }
    }

    @Test
    fun processSamples_whenLeveling_processesImprovedLevelingAndNotifies() {
        val attitudeAvailableListener =
            mockk<AttitudeEstimator.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = AttitudeEstimator(
            context,
            attitudeAvailableListener = attitudeAvailableListener,
            estimateImuLeveling = true
        )

        // set accelerometerSensorAvailable value
        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.sensorAvailable }.returns(true)
        every { accelerometerSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerSensorCollector",
            accelerometerSensorCollectorSpy
        )

        // set gyroscopeSensorAvailable value
        val gyroscopeSensorCollector: GyroscopeSensorCollector? =
            estimator.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        every { gyroscopeSensorCollectorSpy.sensorAvailable }.returns(true)
        every { gyroscopeSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        callPrivateFunc(AttitudeEstimator::class, estimator, "computeType")
        assertEquals(AttitudeEstimator.AttitudeEstimatorType.LEVELING, estimator.type)

        mockkStatic(LevelingEstimator::class)

        assertTrue(estimator.isReady)
        val accelerometerSampleAvailable1: Boolean? =
            estimator.getPrivateProperty("accelerometerSampleAvailable")
        requireNotNull(accelerometerSampleAvailable1)
        assertFalse(accelerometerSampleAvailable1)
        val gyroscopeSampleAvailable1: Boolean? =
            estimator.getPrivateProperty("gyroscopeSampleAvailable")
        requireNotNull(gyroscopeSampleAvailable1)
        assertFalse(gyroscopeSampleAvailable1)
        val magnetometerSampleAvailable1: Boolean? =
            estimator.getPrivateProperty("magnetometerSampleAvailable")
        requireNotNull(magnetometerSampleAvailable1)
        assertFalse(magnetometerSampleAvailable1)

        assertTrue(estimator.start())

        // collect accelerometer measurement
        val accelerometerSampleAvailable2: Boolean? =
            estimator.getPrivateProperty("accelerometerSampleAvailable")
        requireNotNull(accelerometerSampleAvailable2)
        assertFalse(accelerometerSampleAvailable2)
        val gyroscopeSampleAvailable2: Boolean? =
            estimator.getPrivateProperty("gyroscopeSampleAvailable")
        requireNotNull(gyroscopeSampleAvailable2)
        assertFalse(gyroscopeSampleAvailable2)
        val magnetometerSampleAvailable2: Boolean? =
            estimator.getPrivateProperty("magnetometerSampleAvailable")
        requireNotNull(magnetometerSampleAvailable2)
        assertTrue(magnetometerSampleAvailable2)

        val accelerometerMeasurementListener = accelerometerSensorCollector.measurementListener
        requireNotNull(accelerometerMeasurementListener)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val biasX = randomizer.nextFloat()
        val biasY = randomizer.nextFloat()
        val biasZ = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        accelerometerMeasurementListener.onMeasurement(
            ax,
            ay,
            az,
            biasX,
            biasY,
            biasZ,
            timestamp,
            SensorAccuracy.HIGH
        )

        // check
        assertEquals(ax, estimator.getPrivateProperty("ax"))
        assertEquals(ay, estimator.getPrivateProperty("ay"))
        assertEquals(az, estimator.getPrivateProperty("az"))
        val accelerometerSampleAvailable3: Boolean? =
            estimator.getPrivateProperty("accelerometerSampleAvailable")
        requireNotNull(accelerometerSampleAvailable3)
        assertTrue(accelerometerSampleAvailable3)
        val gyroscopeSampleAvailable3: Boolean? =
            estimator.getPrivateProperty("gyroscopeSampleAvailable")
        requireNotNull(gyroscopeSampleAvailable3)
        assertFalse(gyroscopeSampleAvailable3)
        val magnetometerSampleAvailable3: Boolean? =
            estimator.getPrivateProperty("magnetometerSampleAvailable")
        requireNotNull(magnetometerSampleAvailable3)
        assertTrue(magnetometerSampleAvailable3)

        val gyroscopeMeasurementListener = gyroscopeSensorCollector.measurementListener
        requireNotNull(gyroscopeMeasurementListener)

        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        gyroscopeMeasurementListener.onMeasurement(
            wx,
            wy,
            wz,
            biasX,
            biasY,
            biasZ,
            timestamp,
            SensorAccuracy.HIGH
        )

        // check
        assertEquals(ax, estimator.getPrivateProperty("ax"))
        assertEquals(ay, estimator.getPrivateProperty("ay"))
        assertEquals(az, estimator.getPrivateProperty("az"))
        assertEquals(wx, estimator.getPrivateProperty("wx"))
        assertEquals(wy, estimator.getPrivateProperty("wy"))
        assertEquals(wz, estimator.getPrivateProperty("wz"))
        val accelerometerSampleAvailable4: Boolean? =
            estimator.getPrivateProperty("accelerometerSampleAvailable")
        requireNotNull(accelerometerSampleAvailable4)
        assertFalse(accelerometerSampleAvailable4)
        val gyroscopeSampleAvailable4: Boolean? =
            estimator.getPrivateProperty("gyroscopeSampleAvailable")
        requireNotNull(gyroscopeSampleAvailable4)
        assertFalse(gyroscopeSampleAvailable4)
        val magnetometerSampleAvailable4: Boolean? =
            estimator.getPrivateProperty("magnetometerSampleAvailable")
        requireNotNull(magnetometerSampleAvailable4)
        assertFalse(magnetometerSampleAvailable4)

        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        requireNotNull(coordinateTransformation)
        verify(exactly = 1) {
            LevelingEstimator.getAttitude(
                ax.toDouble(),
                ay.toDouble(),
                az.toDouble(),
                wx.toDouble(),
                wy.toDouble(),
                wz.toDouble(),
                coordinateTransformation
            )
        }
        val attitude: Quaternion? = estimator.getPrivateProperty("attitude")
        requireNotNull(attitude)
        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                attitude,
                coordinateTransformation,
                AttitudeEstimator.AttitudeEstimatorType.LEVELING
            )
        }
    }

    private companion object {
        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 100.0

        const val MIN_ACCURACY = 1e-5f
        const val MAX_ACCURACY = 5e-5f

        fun createLocation(): Location {
            val randomizer = UniformRandomizer()
            val location = mockk<Location>()
            every { location.latitude }.returns(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES)
            )
            every { location.longitude }.returns(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES)
            )
            every { location.altitude }.returns(randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT))
            every { location.accuracy }.returns(randomizer.nextFloat(MIN_ACCURACY, MAX_ACCURACY))
            every { location.verticalAccuracyMeters }.returns(
                randomizer.nextFloat(
                    MIN_ACCURACY,
                    MAX_ACCURACY
                )
            )
            return location
        }

    }
}