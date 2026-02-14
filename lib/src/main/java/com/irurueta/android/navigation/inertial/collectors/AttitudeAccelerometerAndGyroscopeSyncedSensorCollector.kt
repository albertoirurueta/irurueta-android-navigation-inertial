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

package com.irurueta.android.navigation.inertial.collectors

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import com.irurueta.android.navigation.inertial.collectors.converters.AccelerometerSensorEventMeasurementConverter
import com.irurueta.android.navigation.inertial.collectors.converters.AttitudeSensorEventMeasurementConverter
import com.irurueta.android.navigation.inertial.collectors.converters.GyroscopeSensorEventMeasurementConverter
import com.irurueta.android.navigation.inertial.collectors.interpolators.AccelerometerSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.collectors.interpolators.AttitudeSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.collectors.interpolators.GyroscopeSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement
import java.util.Queue

/**
 * Collector that synchronizes attitude, accelerometer and gyroscope measurements.
 *
 * @property context Android context.
 * @property windowNanoseconds amount of time to keep measurements buffered expressed in
 * nanoseconds.
 * @property attitudeSensorType One of the supported attitude sensor types.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * @property gyroscopeSensorType One of the supported gyroscope sensor types.
 * @property attitudeSensorDelay sensor delay between samples for attitude sensor.
 * @property accelerometerSensorDelay sensor delay between samples for accelerometer sensor.
 * @property gyroscopeSensorDelay sensor delay between samples for gyroscope sensor.
 * @property primarySensor Indicates the sensor type of the primary sensor being used for
 * measurements synchronization.
 * @property interpolationEnabled indicates whether measurements interpolation is enabled or not.
 * @property measurementListener listener to notify new measurements.
 * @property accuracyChangedListener listener to notify changes in accuracy.
 */
class AttitudeAccelerometerAndGyroscopeSyncedSensorCollector(
    context: Context,
    windowNanoseconds: Long = DEFAULT_WINDOW_NANOSECONDS,
    val attitudeSensorType: AttitudeSensorType = AttitudeSensorType.ABSOLUTE_ATTITUDE,
    val accelerometerSensorType: AccelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
    val gyroscopeSensorType: GyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
    val attitudeSensorDelay: SensorDelay = SensorDelay.FASTEST,
    val accelerometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
    val gyroscopeSensorDelay: SensorDelay = SensorDelay.FASTEST,
    val primarySensor: PrimarySensor = PrimarySensor.GYROSCOPE,
    interpolationEnabled: Boolean = true,
    measurementListener: OnMeasurementListener<AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerAndGyroscopeSyncedSensorCollector>? = null,
    accuracyChangedListener: OnAccuracyChangedListener<AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerAndGyroscopeSyncedSensorCollector>? = null
) : SyncedSensorCollector<AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerAndGyroscopeSyncedSensorCollector>(
    context,
    windowNanoseconds,
    interpolationEnabled,
    measurementListener,
    accuracyChangedListener
) {
    /**
     * Attitude sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     * @see attitudeSensorAvailable
     */
    val attitudeSensor: Sensor? by lazy {
        sensorManager?.getDefaultSensor(attitudeSensorType.value)
    }

    /**
     * Accelerometer sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     * @see accelerometerSensorAvailable
     */
    val accelerometerSensor: Sensor? by lazy {
        sensorManager?.getDefaultSensor(
            accelerometerSensorType.value
        )
    }

    /**
     * Gyroscope sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     * @see gyroscopeSensorAvailable
     */
    val gyroscopeSensor: Sensor? by lazy {
        sensorManager?.getDefaultSensor(
            gyroscopeSensorType.value
        )
    }

    /**
     * Indicates whether requested attitude sensor is available or not.
     */
    val attitudeSensorAvailable: Boolean
        get() = attitudeSensor != null

    /**
     * Indicates whether requested accelerometer sensor is available or not.
     */
    val accelerometerSensorAvailable: Boolean
        get() = accelerometerSensor != null

    /**
     * Indicates whether requested gyroscope sensor is available or not.
     */
    val gyroscopeSensorAvailable: Boolean
        get() = gyroscopeSensor != null

    /**
     * Indicates the sensor type of the primary sensor being used for measurements synchronization.
     */
    override val primarySensorType: Int
        get() {
            return when (primarySensor) {
                PrimarySensor.ATTITUDE -> attitudeSensorType.value
                PrimarySensor.ACCELEROMETER -> accelerometerSensorType.value
                PrimarySensor.GYROSCOPE -> gyroscopeSensorType.value
            }
        }

    /**
     * Instance of measurement being reused and notified after conversion and synchronization of
     * sensor events.
     */
    override val measurement = AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement()

    /**
     * Attitude measurement being reused.
     * This measurement will store interpolated or closest attitude samples from buffers when
     * synchronization is performed.
     */
    private val attitudeMeasurement = AttitudeSensorMeasurement()

    /**
     * Accelerometer measurement being reused.
     * This measurement will store interpolated or closest accelerometer samples from buffers when
     * synchronization is performed.
     */
    private val accelerometerMeasurement = AccelerometerSensorMeasurement()

    /**
     * Gyroscope measurement being reused.
     * This measurement will store interpolated or closest gyroscope samples from buffers when
     * synchronization is performed.
     */
    private val gyroscopeMeasurement = GyroscopeSensorMeasurement()

    /**
     * Attitude measurement interpolator.
     * Performs interpolation or picks closest measurement from buffers.
     */
    private val attitudeInterpolator = AttitudeSensorMeasurementInterpolator()

    /**
     * Accelerometer measurement interpolator.
     * Performs interpolation or picks closest measurement from buffers.
     */
    private val accelerometerInterpolator = AccelerometerSensorMeasurementInterpolator()

    /**
     * Gyroscope measurement interpolator.
     * Performs interpolation or picks closest measurement from buffers.
     */
    private val gyroscopeInterpolator = GyroscopeSensorMeasurementInterpolator()

    /**
     * Sensors being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensors.
     * @see sensorsAvailable
     */
    override val sensors: Collection<Sensor>?
        get() {
            val attitudeSensor = this.attitudeSensor
            val accelerometerSensor = this.accelerometerSensor
            val gyroscopeSensor = this.gyroscopeSensor
            return if (attitudeSensor != null && accelerometerSensor != null && gyroscopeSensor != null) {
                setOf(attitudeSensor, accelerometerSensor, gyroscopeSensor)
            } else {
                null
            }
        }

    /**
     * Gets sensor delay for provided sensor.
     *
     * @param sensor sensor to get delay for.
     * @return sensor delay as an integer value.
     */
    override fun getSensorDelayFor(sensor: Sensor): Int {
        return if (sensor === attitudeSensor) {
            attitudeSensorDelay.value
        } else if (sensor === accelerometerSensor) {
            accelerometerSensorDelay.value
        } else {
            gyroscopeSensorDelay.value
        }
    }

    /**
     * Indicates whether requested sensors are available or not.
     */
    override val sensorsAvailable: Boolean
        get() = attitudeSensorAvailable && accelerometerSensorAvailable && gyroscopeSensorAvailable

    /**
     * Creates a new instance of [SensorMeasurement] from provided [SensorEvent].
     *
     * @param event event to convert from.
     * @return new instance of [SensorMeasurement] or null if conversion failed.
     */
    override fun createSensorMeasurement(event: SensorEvent?): SensorMeasurement<*>? {
        return when (event?.sensor?.type) {
            attitudeSensorType.value -> {
                AttitudeSensorEventMeasurementConverter.convert(event)
            }

            accelerometerSensorType.value -> {
                AccelerometerSensorEventMeasurementConverter.convert(event)
            }

            gyroscopeSensorType.value -> {
                GyroscopeSensorEventMeasurementConverter.convert(event)
            }

            else -> {
                null
            }
        }
    }

    /**
     * Performs interpolation of buffered measurements at a given timestamp when measurement from
     * primary sensor arrives.
     *
     * @param nowNanoSeconds timestamp expressed in nanoseconds from last received measurement in
     * primary sensor.
     */
    @Suppress("UNCHECKED_CAST")
    override fun processSyncedSample(nowNanoSeconds: Long): Boolean {
        val hasAttitude = buffer.containsKey(attitudeSensorType.value)
        val hasAccelerometer = buffer.containsKey(accelerometerSensorType.value)
        val hasGyroscope = buffer.containsKey(gyroscopeSensorType.value)
        if (!hasAttitude || !hasAccelerometer || !hasGyroscope) {
            return false
        }

        val attitudeQueue = buffer[attitudeSensorType.value] as Queue<AttitudeSensorMeasurement>
        val accelerometerQueue =
            buffer[accelerometerSensorType.value] as Queue<AccelerometerSensorMeasurement>
        val gyroscopeQueue = buffer[gyroscopeSensorType.value] as Queue<GyroscopeSensorMeasurement>

        val valid = if (interpolationEnabled) {
            // interpolate measurements from buffers
            attitudeInterpolator.interpolate(
                attitudeQueue,
                nowNanoSeconds,
                attitudeMeasurement
            ) && accelerometerInterpolator.interpolate(
                accelerometerQueue,
                nowNanoSeconds,
                accelerometerMeasurement
            ) && gyroscopeInterpolator.interpolate(
                gyroscopeQueue,
                nowNanoSeconds,
                gyroscopeMeasurement
            )
        } else {
            // pick closest measurements from buffers
            val attitudeMeasurement =
                attitudeInterpolator.findClosest(attitudeQueue, nowNanoSeconds)
            val accelerometerMeasurement =
                accelerometerInterpolator.findClosest(accelerometerQueue, nowNanoSeconds)
            val gyroscopeMeasurement =
                gyroscopeInterpolator.findClosest(gyroscopeQueue, nowNanoSeconds)
            if (attitudeMeasurement != null && accelerometerMeasurement != null && gyroscopeMeasurement != null) {
                attitudeMeasurement.copyTo(this.attitudeMeasurement)
                accelerometerMeasurement.copyTo(this.accelerometerMeasurement)
                gyroscopeMeasurement.copyTo(this.gyroscopeMeasurement)
                true
            } else {
                false
            }
        }

        if (valid) {
            measurement.attitudeMeasurement = attitudeMeasurement
            measurement.accelerometerMeasurement = accelerometerMeasurement
            measurement.gyroscopeMeasurement = gyroscopeMeasurement
            measurement.timestamp = nowNanoSeconds
            return true
        }

        return false
    }

    /**
     * Indicates the sensor type of the primary sensor being used for measurements synchronization.
     */
    enum class PrimarySensor {
        /**
         * Primary sensor is attitude.
         */
        ATTITUDE,

        /**
         * Primary sensor is accelerometer.
         */
        ACCELEROMETER,

        /**
         * Primary sensor is gyroscope.
         */
        GYROSCOPE
    }
}