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
import com.irurueta.android.navigation.inertial.collectors.converters.GravitySensorEventMeasurementConverter
import com.irurueta.android.navigation.inertial.collectors.converters.GyroscopeSensorEventMeasurementConverter
import com.irurueta.android.navigation.inertial.collectors.converters.MagnetometerSensorEventMeasurementConverter
import com.irurueta.android.navigation.inertial.collectors.interpolators.GravitySensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.collectors.interpolators.GyroscopeSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.collectors.interpolators.MagnetometerSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.collectors.measurements.GravityGyroscopeAndMagnetometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GravitySensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement
import java.util.Queue

/**
 * Collector that synchronizes gravity, gyroscope and magnetometer measurements.
 *
 * @property context Android context.
 * @property windowNanoseconds amount of time to keep measurements buffered expressed in
 * nanoseconds.
 * @property gyroscopeSensorType One of the supported gyroscope sensor types.
 * @property magnetometerSensorType One of the supported magnetometer sensor types
 * @property gravitySensorDelay sensor delay between samples for gravity sensor.
 * @property gyroscopeSensorDelay sensor delay between samples for gyroscope sensor.
 * @property magnetometerSensorDelay sensor delay between samples for magnetometer sensor.
 * @property primarySensor Indicates the sensor type of the primary sensor being used for
 * measurements synchronization.
 * @property interpolationEnabled indicates whether measurements interpolation is enabled or not.
 * @property measurementListener listener to notify new measurements.
 * @property gravityAccuracyChangedListener listener to notify changes in accuracy for
 * gravity sensor.
 * @property gyroscopeAccuracyChangedListener listener to notify changes in accuracy for
 * gyroscope sensor.
 * @property magnetometerAccuracyChangedListener listener to notify changes in accuracy for
 * magnetometer sensor.
 */
class GravityGyroscopeAndMagnetometerSyncedSensorCollector(
    context: Context,
    windowNanoseconds: Long = DEFAULT_WINDOW_NANOSECONDS,
    val gyroscopeSensorType: GyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
    val magnetometerSensorType: MagnetometerSensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
    val gravitySensorDelay: SensorDelay = SensorDelay.FASTEST,
    val gyroscopeSensorDelay: SensorDelay = SensorDelay.FASTEST,
    val magnetometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
    val primarySensor: PrimarySensor = PrimarySensor.GYROSCOPE,
    interpolationEnabled: Boolean = true,
    measurementListener: OnMeasurementListener<GravityGyroscopeAndMagnetometerSyncedSensorMeasurement, GravityGyroscopeAndMagnetometerSyncedSensorCollector>? = null,
    var gravityAccuracyChangedListener: OnGravityAccuracyChangedListener? = null,
    var gyroscopeAccuracyChangedListener: OnGyroscopeAccuracyChangedListener? = null,
    var magnetometerAccuracyChangedListener: OnMagnetometerAccuracyChangedListener? = null
) : SyncedSensorCollector<GravityGyroscopeAndMagnetometerSyncedSensorMeasurement, GravityGyroscopeAndMagnetometerSyncedSensorCollector>(
    context,
    windowNanoseconds,
    interpolationEnabled,
    measurementListener
) {

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
     * Magnetometer sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     * @see magnetometerSensorAvailable
     */
    val magnetometerSensor: Sensor? by lazy {
        sensorManager?.getDefaultSensor(
            magnetometerSensorType.value
        )
    }

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
     * Indicates whether requested magnetometer sensor is available or not.
     */
    val magnetometerSensorAvailable: Boolean
        get() = magnetometerSensor != null

    /**
     * Indicates the sensor type of the primary sensor being used for measurements synchronization.
     */
    override val primarySensorType: Int
        get() {
            return when (primarySensor) {
                PrimarySensor.GRAVITY -> Sensor.TYPE_GRAVITY
                PrimarySensor.GYROSCOPE -> gyroscopeSensorType.value
                PrimarySensor.MAGNETOMETER -> magnetometerSensorType.value
            }
        }

    /**
     * Instance of measurement being reused and notified after conversion and synchronization of
     * sensor events.
     */
    override val measurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement()

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
     * Magnetometer measurement being reused.
     * This measurement will store interpolated or closest magnetometer samples from buffers when
     * synchronization is performed.
     */
    private val magnetometerMeasurement = MagnetometerSensorMeasurement()

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
     * Magnetometer measurement interpolator.
     * Performs interpolation or picks closest measurement from buffers.
     */
    private val magnetometerInterpolator = MagnetometerSensorMeasurementInterpolator()

    /**
     * Sensors being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensors.
     * @see sensorsAvailable
     */
    override val sensors: Collection<Sensor>?
        get() {
            val gravitySensor = this.gravitySensor
            val gyroscopeSensor = this.gyroscopeSensor
            val magnetometerSensor = this.magnetometerSensor
            return if (gravitySensor != null && gyroscopeSensor != null && magnetometerSensor != null) {
                setOf(gravitySensor, gyroscopeSensor, magnetometerSensor)
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
        return when (sensor) {
            gravitySensor -> {
                gravitySensorDelay.value
            }
            gyroscopeSensor -> {
                gyroscopeSensorDelay.value
            }
            else -> {
                magnetometerSensorDelay.value
            }
        }
    }

    /**
     * Indicates whether requested sensors are available or not.
     */
    override val sensorsAvailable: Boolean
        get() = gravitySensorAvailable && gyroscopeSensorAvailable && magnetometerSensorAvailable

    /**
     * Creates a new instance of [SensorMeasurement] from provided [SensorEvent].
     *
     * @param event event to convert from.
     * @return new instance of [SensorMeasurement] or null if conversion failed.
     */
    override fun createSensorMeasurement(event: SensorEvent?): SensorMeasurement<*>? {
        return when (event?.sensor?.type) {
            Sensor.TYPE_GRAVITY -> {
                GravitySensorEventMeasurementConverter.convert(event)
            }

            gyroscopeSensorType.value -> {
                GyroscopeSensorEventMeasurementConverter.convert(event)
            }

            magnetometerSensorType.value -> {
                MagnetometerSensorEventMeasurementConverter.convert(event)
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
        val hasGravity = buffer.containsKey(Sensor.TYPE_GRAVITY)
        val hasGyroscope = buffer.containsKey(gyroscopeSensorType.value)
        val hasMagnetometer = buffer.containsKey(magnetometerSensorType.value)
        if (!hasGravity || !hasGyroscope || !hasMagnetometer) {
            return false
        }

        val gravityQueue = buffer[Sensor.TYPE_GRAVITY] as Queue<GravitySensorMeasurement>
        val gyroscopeQueue = buffer[gyroscopeSensorType.value] as Queue<GyroscopeSensorMeasurement>
        val magnetometerQueue =
            buffer[magnetometerSensorType.value] as Queue<MagnetometerSensorMeasurement>

        val valid = if (interpolationEnabled) {
            // interpolate measurements from buffers
            gravityInterpolator.interpolate(
                gravityQueue,
                nowNanoSeconds,
                gravityMeasurement
            ) && gyroscopeInterpolator.interpolate(
                gyroscopeQueue,
                nowNanoSeconds,
                gyroscopeMeasurement
            ) && magnetometerInterpolator.interpolate(
                magnetometerQueue,
                nowNanoSeconds,
                magnetometerMeasurement
            )
        } else {
            // pick closest measurements from buffers
            val gravityMeasurement =
                gravityInterpolator.findClosest(gravityQueue, nowNanoSeconds)
            val gyroscopeMeasurement =
                gyroscopeInterpolator.findClosest(gyroscopeQueue, nowNanoSeconds)
            val magnetometerMeasurement =
                magnetometerInterpolator.findClosest(magnetometerQueue, nowNanoSeconds)
            if (gravityMeasurement != null && gyroscopeMeasurement != null && magnetometerMeasurement != null) {
                gravityMeasurement.copyTo(this.gravityMeasurement)
                gyroscopeMeasurement.copyTo(this.gyroscopeMeasurement)
                magnetometerMeasurement.copyTo(this.magnetometerMeasurement)
                true
            } else {
                false
            }
        }

        if (valid) {
            measurement.gravityMeasurement = gravityMeasurement
            measurement.gyroscopeMeasurement = gyroscopeMeasurement
            measurement.magnetometerMeasurement = magnetometerMeasurement
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
        if (sensor === gravitySensor) {
            gravityAccuracyChangedListener?.onAccuracyChanged(this, sensorAccuracy)
        } else if (sensor === gyroscopeSensor) {
            gyroscopeAccuracyChangedListener?.onAccuracyChanged(this, sensorAccuracy)
        } else if (sensor === magnetometerSensor) {
            magnetometerAccuracyChangedListener?.onAccuracyChanged(this, sensorAccuracy)
        }
    }

    /**
     * Indicates the sensor type of the primary sensor being used for measurements synchronization.
     */
    enum class PrimarySensor {
        /**
         * Primary sensor is gravity.
         */
        GRAVITY,

        /**
         * Primary sensor is gyroscope.
         */
        GYROSCOPE,

        /**
         * Primary sensor is magnetometer.
         */
        MAGNETOMETER
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
            collector: GravityGyroscopeAndMagnetometerSyncedSensorCollector,
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
            collector: GravityGyroscopeAndMagnetometerSyncedSensorCollector,
            accuracy: SensorAccuracy?
        )
    }

    /**
     * Interface to notify when magnetometer sensor accuracy changes.
     */
    fun interface OnMagnetometerAccuracyChangedListener {
        /**
         * Called when accuracy changes.
         *
         * @param collector collector that raised this event.
         * @param accuracy new accuracy.
         */
        fun onAccuracyChanged(
            collector: GravityGyroscopeAndMagnetometerSyncedSensorCollector,
            accuracy: SensorAccuracy?
        )
    }
}