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
import com.irurueta.android.navigation.inertial.collectors.converters.MagnetometerSensorEventMeasurementConverter
import com.irurueta.android.navigation.inertial.collectors.interpolators.AccelerometerSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.collectors.interpolators.GravitySensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.collectors.interpolators.MagnetometerSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerGravityAndMagnetometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.GravitySensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement
import java.util.Queue

/**
 * Collector that synchronizes accelerometer, gravity and magnetometer measurements.
 *
 * @property context Android context.
 * @property windowNanoseconds amount of time to keep measurements buffered expressed in
 * nanoseconds.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * @property magnetometerSensorType One of the supported magnetometer sensor types.
 * @property accelerometerSensorDelay sensor delay between samples for accelerometer sensor.
 * @property gravitySensorDelay sensor delay between samples for gravity sensor.
 * @property magnetometerSensorDelay sensor delay between samples for magnetometer sensor.
 * @property primarySensor Indicates the sensor type of the primary sensor being used for
 * measurements synchronization.
 * @property interpolationEnabled indicates whether measurements interpolation is enabled or not.
 * @property measurementListener listener to notify new measurements.
 * @property accelerometerAccuracyChangedListener listener to notify changes in accuracy for
 * accelerometer sensor.
 * @property gravityAccuracyChangedListener listener to notify changes in accuracy for
 * gravity sensor.
 * @property magnetometerAccuracyChangedListener listener to notify changes in accuracy for
 * magnetometer sensor.
 */
class AccelerometerGravityAndMagnetometerSyncedSensorCollector(
    context: Context,
    windowNanoseconds: Long = DEFAULT_WINDOW_NANOSECONDS,
    val accelerometerSensorType: AccelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
    val magnetometerSensorType: MagnetometerSensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
    val accelerometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
    val gravitySensorDelay: SensorDelay = SensorDelay.FASTEST,
    val magnetometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
    val primarySensor: PrimarySensor = PrimarySensor.ACCELEROMETER,
    interpolationEnabled: Boolean = true,
    measurementListener: OnMeasurementListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSyncedSensorCollector>? = null,
    var accelerometerAccuracyChangedListener: OnAccelerometerAccuracyChangedListener? = null,
    var gravityAccuracyChangedListener: OnGravityAccuracyChangedListener? = null,
    var magnetometerAccuracyChangedListener: OnMagnetometerAccuracyChangedListener? = null
) : SyncedSensorCollector<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSyncedSensorCollector>(
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
                PrimarySensor.ACCELEROMETER -> accelerometerSensorType.value
                PrimarySensor.GRAVITY -> Sensor.TYPE_GRAVITY
                PrimarySensor.MAGNETOMETER -> magnetometerSensorType.value
            }
        }

    /**
     * Instance of measurement being reused and notified after conversion and synchronization of
     * sensor events.
     */
    override val measurement = AccelerometerGravityAndMagnetometerSyncedSensorMeasurement()

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
     * Magnetometer measurement being reused.
     * This measurement will store interpolated or closest magnetometer samples from buffers when
     * synchronization is performed.
     */
    private val magnetometerMeasurement = MagnetometerSensorMeasurement()

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
            val accelerometerSensor = this.accelerometerSensor
            val gravitySensor = this.gravitySensor
            val magnetometerSensor = this.magnetometerSensor
            return if (accelerometerSensor != null && gravitySensor != null && magnetometerSensor != null) {
                setOf(accelerometerSensor, gravitySensor, magnetometerSensor)
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
            magnetometerSensorDelay.value
        }
    }

    /**
     * Indicates whether requested sensors are available or not.
     */
    override val sensorsAvailable: Boolean
        get() = accelerometerSensorAvailable && gravitySensorAvailable && magnetometerSensorAvailable

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
        val hasAccelerometer = buffer.containsKey(accelerometerSensorType.value)
        val hasGravity = buffer.containsKey(Sensor.TYPE_GRAVITY)
        val hasMagnetometer = buffer.containsKey(magnetometerSensorType.value)
        if (!hasAccelerometer || !hasGravity || !hasMagnetometer) {
            return false
        }

        val accelerometerQueue =
            buffer[accelerometerSensorType.value] as Queue<AccelerometerSensorMeasurement>
        val gravityQueue = buffer[Sensor.TYPE_GRAVITY] as Queue<GravitySensorMeasurement>
        val magnetometerQueue = buffer[magnetometerSensorType.value] as Queue<MagnetometerSensorMeasurement>

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
            ) && magnetometerInterpolator.interpolate(
                magnetometerQueue,
                nowNanoSeconds,
                magnetometerMeasurement
            )
        } else {
            // pick closest measurements from buffers
            val accelerometerMeasurement =
                accelerometerInterpolator.findClosest(accelerometerQueue, nowNanoSeconds)
            val gravityMeasurement =
                gravityInterpolator.findClosest(gravityQueue, nowNanoSeconds)
            val magnetometerMeasurement =
                magnetometerInterpolator.findClosest(magnetometerQueue, nowNanoSeconds)
            if (accelerometerMeasurement != null && gravityMeasurement != null && magnetometerMeasurement != null) {
                accelerometerMeasurement.copyTo(this.accelerometerMeasurement)
                gravityMeasurement.copyTo(this.gravityMeasurement)
                magnetometerMeasurement.copyTo(this.magnetometerMeasurement)
                true
            } else {
                false
            }
        }

        if (valid) {
            measurement.accelerometerMeasurement = accelerometerMeasurement
            measurement.gravityMeasurement = gravityMeasurement
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
        if (sensor === accelerometerSensor) {
            accelerometerAccuracyChangedListener?.onAccuracyChanged(this, sensorAccuracy)
        } else if (sensor === gravitySensor) {
            gravityAccuracyChangedListener?.onAccuracyChanged(this, sensorAccuracy)
        } else if (sensor === magnetometerSensor) {
            magnetometerAccuracyChangedListener?.onAccuracyChanged(this, sensorAccuracy)
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
         * Primary sensor is magnetometer.
         */
        MAGNETOMETER
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
            collector: AccelerometerGravityAndMagnetometerSyncedSensorCollector,
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
            collector: AccelerometerGravityAndMagnetometerSyncedSensorCollector,
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
            collector: AccelerometerGravityAndMagnetometerSyncedSensorCollector,
            accuracy: SensorAccuracy?
        )
    }
}