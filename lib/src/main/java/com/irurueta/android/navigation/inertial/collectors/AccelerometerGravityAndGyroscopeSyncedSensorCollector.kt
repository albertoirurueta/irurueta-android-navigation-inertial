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
import com.irurueta.android.navigation.inertial.collectors.converters.GravitySensorEventMeasurementConverter
import com.irurueta.android.navigation.inertial.collectors.converters.GyroscopeSensorEventMeasurementConverter
import com.irurueta.android.navigation.inertial.collectors.interpolators.AccelerometerSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.collectors.interpolators.GravitySensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.collectors.interpolators.GyroscopeSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerGravityAndGyroscopeSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.GravitySensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement
import java.util.Queue

/**
 * Collector that synchronizes accelerometer, gravity and gyroscope measurements.
 *
 * @property context Android context.
 * @property windowNanoseconds amount of time to keep measurements buffered expressed in
 * nanoseconds.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * @property gyroscopeSensorType One of the supported gyroscope sensor types.
 * @property accelerometerSensorDelay sensor delay between samples for accelerometer sensor.
 * @property gravitySensorDelay sensor delay between samples for gravity sensor.
 * @property gyroscopeSensorDelay sensor delay between samples for gyroscope sensor.
 * @property primarySensor Indicates the sensor type of the primary sensor being used for
 * measurements synchronization.
 * @property interpolationEnabled indicates whether measurements interpolation is enabled or not.
 * @property measurementListener listener to notify new measurements. It must be noticed that
 * measurements notification might be delayed.
 * @property accelerometerAccuracyChangedListener listener to notify changes in accuracy for
 * accelerometer sensor.
 * @property gravityAccuracyChangedListener listener to notify changes in accuracy for
 * gravity sensor.
 * @property gyroscopeAccuracyChangedListener listener to notify changes in accuracy for
 * gyroscope sensor.
 */
class AccelerometerGravityAndGyroscopeSyncedSensorCollector(
    context: Context,
    windowNanoseconds: Long = DEFAULT_WINDOW_NANOSECONDS,
    val accelerometerSensorType: AccelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
    val gyroscopeSensorType: GyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
    val accelerometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
    val gravitySensorDelay: SensorDelay = SensorDelay.FASTEST,
    val gyroscopeSensorDelay: SensorDelay = SensorDelay.FASTEST,
    val primarySensor: PrimarySensor = PrimarySensor.GYROSCOPE,
    interpolationEnabled: Boolean = true,
    measurementListener: OnMeasurementListener<AccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AccelerometerGravityAndGyroscopeSyncedSensorCollector>? = null,
    var accelerometerAccuracyChangedListener: OnAccelerometerAccuracyChangedListener? = null,
    var gravityAccuracyChangedListener: OnGravityAccuracyChangedListener? = null,
    var gyroscopeAccuracyChangedListener: OnGyroscopeAccuracyChangedListener? = null
) : SyncedSensorCollector<AccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AccelerometerGravityAndGyroscopeSyncedSensorCollector>(
    context,
    windowNanoseconds,
    interpolationEnabled,
    measurementListener
) {

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
     * Sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     * @see gravitySensorAvailable
     */
    val gravitySensor: Sensor? by lazy { sensorManager?.getDefaultSensor(Sensor.TYPE_GRAVITY) }

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
     * Indicates whether requested accelerometer sensor is available or not.
     */
    val accelerometerSensorAvailable: Boolean
        get() = accelerometerSensor != null

    /**
     * Indicates whether requested gravity sensor is available or not.
     */
    val gravitySensorAvailable: Boolean
        get() = gravitySensor != null

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
                PrimarySensor.ACCELEROMETER -> accelerometerSensorType.value
                PrimarySensor.GRAVITY -> Sensor.TYPE_GRAVITY
                PrimarySensor.GYROSCOPE -> gyroscopeSensorType.value
            }
        }

    /**
     * Instance of measurement being reused and notified after conversion and synchronization of
     * sensor events.
     */
    override val measurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement()

    /**
     * Accelerometer measurement being reused.
     * This measurement will store interpolated or closest accelerometer samples from buffers when
     * synchronization is performed.
     */
    private val accelerometerMeasurement = AccelerometerSensorMeasurement()

    /**
     * Gravity measurement being reused.
     * This measurement will store interpolated or closest gravity samples from buffers when
     * synchronization is performed.
     */
    private val gravityMeasurement = GravitySensorMeasurement()

    /**
     * Gyroscope measurement being reused.
     * This measurement will store interpolated or closest gyroscope samples from buffers when
     * synchronization is performed.
     */
    private val gyroscopeMeasurement = GyroscopeSensorMeasurement()

    /**
     * Accelerometer measurement interpolator.
     * Performs interpolation or picks closest measurement from buffers.
     */
    private val accelerometerInterpolator = AccelerometerSensorMeasurementInterpolator()

    /**
     * Gravity measurement interpolator.
     * Performs interpolation or picks closest measurement from buffers.
     */
    private val gravityInterpolator = GravitySensorMeasurementInterpolator()

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
            val accelerometerSensor = this.accelerometerSensor
            val gravitySensor = this.gravitySensor
            val gyroscopeSensor = this.gyroscopeSensor
            return if (accelerometerSensor != null && gravitySensor != null && gyroscopeSensor != null) {
                setOf(accelerometerSensor, gravitySensor, gyroscopeSensor)
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
        return if (sensor === accelerometerSensor) {
            accelerometerSensorDelay.value
        } else if (sensor === gravitySensor) {
            gravitySensorDelay.value
        } else {
            gyroscopeSensorDelay.value
        }
    }

    /**
     * Indicates whether requested sensors are available or not.
     */
    override val sensorsAvailable: Boolean
        get() = accelerometerSensorAvailable && gravitySensorAvailable && gyroscopeSensorAvailable

    /**
     * Creates a new instance of [SensorMeasurement] from provided [SensorEvent].
     *
     * @param event event to convert from.
     * @return new instance of [SensorMeasurement] or null if conversion failed.
     */
    override fun createSensorMeasurement(event: SensorEvent?): SensorMeasurement<*>? {
        return when (event?.sensor?.type) {
            accelerometerSensorType.value -> {
                AccelerometerSensorEventMeasurementConverter.convert(event)
            }

            Sensor.TYPE_GRAVITY -> {
                GravitySensorEventMeasurementConverter.convert(event)
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
        val hasAccelerometer = buffer.containsKey(accelerometerSensorType.value)
        val hasGravity = buffer.containsKey(Sensor.TYPE_GRAVITY)
        val hasGyroscope = buffer.containsKey(gyroscopeSensorType.value)
        if (!hasAccelerometer || !hasGravity || !hasGyroscope) {
            return false
        }

        val accelerometerQueue =
            buffer[accelerometerSensorType.value] as Queue<AccelerometerSensorMeasurement>
        val gravityQueue = buffer[Sensor.TYPE_GRAVITY] as Queue<GravitySensorMeasurement>
        val gyroscopeQueue = buffer[gyroscopeSensorType.value] as Queue<GyroscopeSensorMeasurement>

        val valid = if (interpolationEnabled) {
            // interpolate measurements from buffers
            accelerometerInterpolator.interpolate(
                accelerometerQueue,
                nowNanoSeconds,
                accelerometerMeasurement
            ) && gravityInterpolator.interpolate(
                gravityQueue,
                nowNanoSeconds,
                gravityMeasurement
            ) && gyroscopeInterpolator.interpolate(
                gyroscopeQueue,
                nowNanoSeconds,
                gyroscopeMeasurement
            )
        } else {
            // pick closest measurements from buffers
            val accelerometerMeasurement =
                accelerometerInterpolator.findClosest(accelerometerQueue, nowNanoSeconds)
            val gravityMeasurement =
                gravityInterpolator.findClosest(gravityQueue, nowNanoSeconds)
            val gyroscopeMeasurement =
                gyroscopeInterpolator.findClosest(gyroscopeQueue, nowNanoSeconds)
            if (accelerometerMeasurement != null && gravityMeasurement != null && gyroscopeMeasurement != null) {
                accelerometerMeasurement.copyTo(this.accelerometerMeasurement)
                gravityMeasurement.copyTo(this.gravityMeasurement)
                gyroscopeMeasurement.copyTo(this.gyroscopeMeasurement)
                true
            } else {
                false
            }
        }

        if (valid) {
            measurement.accelerometerMeasurement = accelerometerMeasurement
            measurement.gravityMeasurement = gravityMeasurement
            measurement.gyroscopeMeasurement = gyroscopeMeasurement
            measurement.timestamp = nowNanoSeconds
            return true
        }

        return false
    }

    /**
     * Processes accuracy changed event for proper notification.
     *
     * @param sensor sensor whose accuracy has changed.
     * @param accuracy new accuracy.
     */
    override fun processAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        if (sensor == null) {
            return
        }

        val sensorAccuracy = SensorAccuracy.from(accuracy) ?: return
        if (sensor === accelerometerSensor) {
            accelerometerAccuracyChangedListener?.onAccuracyChanged(this, sensorAccuracy)
        } else if (sensor === gravitySensor) {
            gravityAccuracyChangedListener?.onAccuracyChanged(this, sensorAccuracy)
        } else if (sensor === gyroscopeSensor) {
            gyroscopeAccuracyChangedListener?.onAccuracyChanged(this, sensorAccuracy)
        }
    }

    /**
     * Indicates the sensor type of the primary sensor being used for measurements synchronization.
     */
    enum class PrimarySensor {
        /**
         * Primary sensor is accelerometer.
         */
        ACCELEROMETER,

        /**
         * Primary sensor is gravity.
         */
        GRAVITY,

        /**
         * Primary sensor is gyroscope.
         */
        GYROSCOPE
    }

    /**
     * Interface to notify when accelerometer sensor accuracy changes.
     */
    fun interface OnAccelerometerAccuracyChangedListener {

        /**
         * Called when accuracy changes.
         *
         * @param collector collector that raised this event.
         * @param accuracy new accuracy.
         */
        fun onAccuracyChanged(
            collector: AccelerometerGravityAndGyroscopeSyncedSensorCollector,
            accuracy: SensorAccuracy?
        )
    }

    /**
     * Interface to notify when gravity sensor accuracy changes.
     */
    fun interface OnGravityAccuracyChangedListener {
        /**
         * Called when accuracy changes.
         *
         * @param collector collector that raised this event.
         * @param accuracy new accuracy.
         */
        fun onAccuracyChanged(
            collector: AccelerometerGravityAndGyroscopeSyncedSensorCollector,
            accuracy: SensorAccuracy?
        )
    }

    /**
     * Interface to notify when gyroscope sensor accuracy changes.
     */
    fun interface OnGyroscopeAccuracyChangedListener {

        /**
         * Called when accuracy changes.
         *
         * @param collector collector that raised this event.
         * @param accuracy new accuracy.
         */
        fun onAccuracyChanged(
            collector: AccelerometerGravityAndGyroscopeSyncedSensorCollector,
            accuracy: SensorAccuracy?
        )
    }
}