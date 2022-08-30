/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial.estimators

import android.content.Context
import android.location.Location
import com.irurueta.android.navigation.inertial.ENUtoNEDTriadConverter
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GravitySensorCollector
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.frames.NEDPosition
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.navigation.inertial.estimators.AttitudeEstimator
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.units.MagneticFluxDensityConverter
import java.util.*

/**
 * Estimates a leveled absolute attitude using accelerometer (or gravity) and magnetometer sensors.
 * Gyroscope is not used in this estimator.
 * Roll and pitch Euler angles are leveled using accelerometer or gravity sensors.
 * Yaw angle is obtained from magnetometer once the leveling is estimated.
 *
 * @property context Android context.
 * @property sensorDelay Delay of sensors between samples.
 * @property useAccelerometer true to use accelerometer sensor, false to use system gravity sensor
 * for leveling purposes.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * (Only used if [useAccelerometer] is true).
 * @property magnetometerSensorType One of the supported magnetometer sensor types.
 * @property accelerometerAveragingFilter an averaging filter for accelerometer samples to obtain
 * sensed gravity component of specific force. (Only used if [useAccelerometer] is true).
 * @property timestamp Timestamp when World Magnetic Model will be evaluated to obtain current.
 * Only taken into account if [useWorldMagneticModel] is tue.
 * magnetic declination. Only taken into account if [useWorldMagneticModel] is true.
 * @property estimateCoordinateTransformation true to estimate coordinate transformation, false
 * otherwise. If not needed, it can be disabled to improve performance and decrease cpu load.
 * @property estimateDisplayEulerAngles true to estimate euler angles, false otherwise. If not
 * needed, it can be disabled to improve performance and decrease cpu load.
 * @property attitudeAvailableListener listener to notify when a new attitude measurement is
 * available.
 * @property magnetometerMeasurementListener listener to notify new magnetometer measurements.
 */
class GeomagneticAttitudeEstimator private constructor(
    val context: Context,
    val sensorDelay: SensorDelay,
    val useAccelerometer: Boolean,
    val accelerometerSensorType: AccelerometerSensorCollector.SensorType,
    val magnetometerSensorType: MagnetometerSensorCollector.SensorType,
    val accelerometerAveragingFilter: AveragingFilter,
    var timestamp: Date,
    val estimateCoordinateTransformation: Boolean,
    val estimateDisplayEulerAngles: Boolean,
    var attitudeAvailableListener: OnAttitudeAvailableListener?,
    var magnetometerMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener?
) {

    /**
     * Constructor.
     *
     * @param context Android context.
     * @param location Device location.
     * @param sensorDelay Delay of sensors between samples.
     * @param useAccelerometer true to use accelerometer sensor, false to use system gravity sensor
     * for leveling purposes.
     * @param accelerometerSensorType One of the supported accelerometer sensor types.
     * (Only used if [useAccelerometer] is true).
     * @param magnetometerSensorType One of the supported magnetometer sensor types.
     * @param accelerometerAveragingFilter an averaging filter for accelerometer samples to obtain
     * sensed gravity component of specific force. (Only used if [useAccelerometer] is true).
     * @param worldMagneticModel Earth's magnetic model. Null to use default model
     * when [useWorldMagneticModel] is true. If [useWorldMagneticModel] is false, this is ignored.
     * @param timestamp Timestamp when World Magnetic Model will be evaluated to obtain current.
     * Only taken into account if [useWorldMagneticModel] is tue.
     * @param useWorldMagneticModel true so that world magnetic model is taken into account to
     * adjust attitude yaw angle by current magnetic declination based on current World Magnetic
     * Model, location and timestamp, false to ignore declination.
     * @param useAccurateLevelingEstimator true to use accurate leveling, false to use a normal one.
     * @param estimateCoordinateTransformation true to estimate coordinate transformation, false
     * otherwise. If not needed, it can be disabled to improve performance and decrease cpu load.
     * @param estimateDisplayEulerAngles true to estimate euler angles, false otherwise. If not
     * needed, it can be disabled to improve performance and decrease cpu load.
     * @param attitudeAvailableListener listener to notify when a new attitude measurement is
     * available.
     * @param accelerometerMeasurementListener listener to notify new accelerometer measurements.
     * (Only used if [useAccelerometer] is true).
     * @param gravityMeasurementListener listener to notify new gravity measurements.
     * (Only used if [useAccelerometer] is false).
     * @param magnetometerMeasurementListener listener to notify new magnetometer measurements.
     * @param gravityEstimationListener listener to notify when a new gravity estimation is
     * available.
     */
    constructor(
        context: Context,
        location: Location? = null,
        sensorDelay: SensorDelay = SensorDelay.GAME,
        useAccelerometer: Boolean = false,
        accelerometerSensorType: AccelerometerSensorCollector.SensorType =
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
        magnetometerSensorType: MagnetometerSensorCollector.SensorType =
            MagnetometerSensorCollector.SensorType.MAGNETOMETER,
        accelerometerAveragingFilter: AveragingFilter = LowPassAveragingFilter(),
        worldMagneticModel: WorldMagneticModel? = null,
        timestamp: Date = Date(),
        useWorldMagneticModel: Boolean = false,
        useAccurateLevelingEstimator: Boolean = false,
        estimateCoordinateTransformation: Boolean = false,
        estimateDisplayEulerAngles: Boolean = true,
        attitudeAvailableListener: OnAttitudeAvailableListener? = null,
        accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? = null,
        gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener? = null,
        magnetometerMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? = null,
        gravityEstimationListener: GravityEstimator.OnEstimationListener? = null
    ) : this(
        context,
        sensorDelay,
        useAccelerometer,
        accelerometerSensorType,
        magnetometerSensorType,
        accelerometerAveragingFilter,
        timestamp,
        estimateCoordinateTransformation,
        estimateDisplayEulerAngles,
        attitudeAvailableListener,
        magnetometerMeasurementListener
    ) {
        this.location = location
        this.useAccurateLevelingEstimator = useAccurateLevelingEstimator
        this.worldMagneticModel = worldMagneticModel
        this.useWorldMagneticModel = useWorldMagneticModel
        this.accelerometerMeasurementListener = accelerometerMeasurementListener
        this.gravityMeasurementListener = gravityMeasurementListener
        this.gravityEstimationListener = gravityEstimationListener
    }


    /**
     * Internal leveling estimator.
     */
    private lateinit var levelingEstimator: BaseLevelingEstimator<*, *>

    /**
     * Estimator of magnetic declination of attitude yaw angle if world magnetic model is taken
     * into account.
     */
    private var wmmEstimator: WMMEarthMagneticFluxDensityEstimator? = null

    /**
     * Position expressed in NED coordinates being reused.
     */
    private val position = NEDPosition()

    /**
     * Instance to be reused which contains attitude of internal leveling estimator.
     */
    private val levelingAttitude = Quaternion()

    /**
     * Instance to be reused containing estimated attitude in NED coordinates by merging leveled
     * attitude with magnetometer measurements.
     */
    private val fusedAttitude = Quaternion()

    /**
     * Triad to be reused for ENU to NED coordinates conversion.
     */
    private val triad = MagneticFluxDensityTriad()

    /**
     * Internal magnetometer sensor collector.
     */
    private val magnetometerSensorCollector = MagnetometerSensorCollector(
        context,
        magnetometerSensorType,
        sensorDelay,
        { bx, by, bz, hardIronX, hardIronY, hardIronZ, timestamp, accuracy ->
            magnetometerMeasurementListener?.onMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                accuracy
            )

            val sensorBx = MagneticFluxDensityConverter.microTeslaToTesla(
                (bx - (hardIronX ?: 0.0f)).toDouble()
            )
            val sensorBy = MagneticFluxDensityConverter.microTeslaToTesla(
                (by - (hardIronY ?: 0.0f)).toDouble()
            )
            val sensorBz = MagneticFluxDensityConverter.microTeslaToTesla(
                (bz - (hardIronZ ?: 0.0f)).toDouble()
            )

            ENUtoNEDTriadConverter.convert(sensorBx, sensorBy, sensorBz, triad)

            hasMagnetometerValues = true
        })

    /**
     * Array to be reused containing euler angles of leveling attitude.
     */
    private val displayEulerAngles = DoubleArray(Quaternion.N_ANGLES)

    /**
     * Instance to be reused containing coordinate transformation in NED coordinates.
     */
    private val coordinateTransformation =
        CoordinateTransformation(FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME)

    /**
     * Indicates whether magnetometer sensor values have been received or not.
     */
    private var hasMagnetometerValues = false

    /**
     * Gets or sets device location
     *
     * @throws IllegalStateException if estimator is running and a null value is set.
     */
    var location: Location? = null
        @Throws(IllegalStateException::class)
        set(value) {
            check(value != null || !running)
            field = value
        }

    /**
     * Indicates whether accurate leveling must be used or not.
     *
     * @throws IllegalStateException if estimator is running.
     */
    var useAccurateLevelingEstimator: Boolean = false
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            if (value) {
                checkNotNull(location)
            }

            field = value
            buildLevelingEstimator()
        }

    /**
     * Earth's magnetic model. If null, the default model is used if [useWorldMagneticModel] is
     * true. If [useWorldMagneticModel] is false, this is ignored.
     *
     * @throws IllegalStateException if estimator is running.
     */
    var worldMagneticModel: WorldMagneticModel? = null
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)

            field = value
            buildWMMEstimator()
        }

    /**
     * Indicates whether world magnetic model is taken into account to adjust attitude yaw angle by
     * current magnetic declination based on current World Magnetic Model, location and timestamp.
     */
    var useWorldMagneticModel: Boolean = false
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)

            field = value
            buildWMMEstimator()
        }

    /**
     * Listener to notify new accelerometer measurements.
     * (Only used if [useAccelerometer] is true).
     */
    var accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? = null
        set(value) {
            field = value
            levelingEstimator.accelerometerMeasurementListener = value
        }

    /**
     * Listener to notify new gravity measurements.
     * (Only used if [useAccelerometer] is false).
     */
    var gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener? = null
        set(value) {
            field = value
            levelingEstimator.gravityMeasurementListener = value
        }

    /**
     * Listener to notify when a new gravity estimation is
     * available.
     */
    var gravityEstimationListener: GravityEstimator.OnEstimationListener? = null
        set(value) {
            field = value
            levelingEstimator.gravityEstimationListener = value
        }

    /**
     * Indicates whether this estimator is running or not.
     */
    var running: Boolean = false
        private set

    /**
     * Starts this estimator.
     *
     * @return true if estimator successfully started, false otherwise.
     * @throws IllegalStateException if estimator is already running.
     */
    @Throws(IllegalStateException::class)
    fun start(): Boolean {
        check(!running)

        reset()
        running = levelingEstimator.start() && magnetometerSensorCollector.start()
        if (!running) {
            stop()
        }
        return running
    }

    /**
     * Stops this estimator.
     */
    fun stop() {
        levelingEstimator.stop()
        magnetometerSensorCollector.stop()
        running = false
    }

    /**
     * Resets internal parameters.
     */
    private fun reset() {
        hasMagnetometerValues = false
    }

    /**
     * Builds the internal leveling estimator.
     */
    private fun buildLevelingEstimator() {
        levelingEstimator = if (useAccurateLevelingEstimator) {

            val location = this.location
            checkNotNull(location)
            AccurateLevelingEstimator(
                context,
                location,
                sensorDelay,
                useAccelerometer,
                accelerometerSensorType,
                accelerometerAveragingFilter,
                estimateCoordinateTransformation = false,
                estimateDisplayEulerAngles = false,
                levelingAvailableListener = { _, attitude, _, _, _ ->
                    processLeveling(attitude)
                },
                gravityEstimationListener = gravityEstimationListener,
                accelerometerMeasurementListener = accelerometerMeasurementListener,
                gravityMeasurementListener = gravityMeasurementListener
            )
        } else {
            LevelingEstimator(
                context,
                sensorDelay,
                useAccelerometer,
                accelerometerSensorType,
                accelerometerAveragingFilter,
                estimateCoordinateTransformation = false,
                estimateDisplayEulerAngles = false,
                levelingAvailableListener = { _, attitude, _, _, _ ->
                    processLeveling(attitude)
                },
                gravityEstimationListener = gravityEstimationListener,
                accelerometerMeasurementListener = accelerometerMeasurementListener,
                gravityMeasurementListener = gravityMeasurementListener
            )
        }
    }

    /**
     * Builds World Magnetic Model if needed.
     */
    private fun buildWMMEstimator() {
        wmmEstimator = if (useWorldMagneticModel) {
            val model = worldMagneticModel
            if (model != null) {
                WMMEarthMagneticFluxDensityEstimator(model)
            } else {
                WMMEarthMagneticFluxDensityEstimator()
            }
        } else {
            null
        }
    }

    /**
     * Processes last received leveling attitude to obtain an updated absolute attitude taking
     * into account last magnetometer measurement.
     */
    private fun processLeveling(attitude: Quaternion) {
        if (!hasMagnetometerValues) {
            return
        }

        attitude.copyTo(levelingAttitude)

        // obtain roll and pitch euler angles from leveling
        levelingAttitude.toEulerAngles(displayEulerAngles)
        val roll = displayEulerAngles[0]
        val pitch = displayEulerAngles[1]
        val declination = getDeclination()
        val yaw = AttitudeEstimator.getYaw(triad.valueX, triad.valueY, triad.valueZ, declination, roll, pitch)

        fusedAttitude.setFromEulerAngles(roll, pitch, yaw)

        val c: CoordinateTransformation? =
            if (estimateCoordinateTransformation) {
                coordinateTransformation.fromRotation(fusedAttitude)
                coordinateTransformation
            } else {
                null
            }

        val displayRoll: Double?
        val displayPitch: Double?
        val displayYaw: Double?
        if (estimateDisplayEulerAngles) {
            fusedAttitude.toEulerAngles(displayEulerAngles)
            displayRoll = displayEulerAngles[0]
            displayPitch = displayEulerAngles[1]
            displayYaw = displayEulerAngles[2]
        } else {
            displayRoll = null
            displayPitch = null
            displayYaw = null
        }

        // notify
        attitudeAvailableListener?.onAttitudeAvailable(
            this@GeomagneticAttitudeEstimator,
            fusedAttitude,
            displayRoll,
            displayPitch,
            displayYaw,
            c
        )
    }

    /**
     * Obtains magnetic declination expressed in radians if World Magnetic Model is used,
     * otherwise zero is returned.
     */
    private fun getDeclination(): Double {
        val wmmEstimator = this.wmmEstimator ?: return 0.0
        val location = this.location ?: return 0.0

        location.toNEDPosition(position)
        return wmmEstimator.getDeclination(
            position.latitude, position.longitude, position.height, timestamp
        )
    }

    /**
     * Interface to notify when a new attitude measurement is available.
     */
    fun interface OnAttitudeAvailableListener {

        /**
         * Called when a new attitude measurement is available.
         *
         * @param estimator attitude estimator that raised this event.
         * @param attitude attitude expressed in NED coordinates.
         * @param roll roll angle expressed in radians. Only available if
         * [estimateDisplayEulerAngles] is true.
         * @param pitch pitch angle expressed in radians. Only available if
         * [estimateDisplayEulerAngles] is true.
         * @param yaw yaw angle expressed in radians. Only available if
         * [estimateDisplayEulerAngles] is true.
         * @param coordinateTransformation coordinate transformation containing measured leveled
         * geomagnetic attitude. Only available if [estimateCoordinateTransformation] is true.
         */
        fun onAttitudeAvailable(
            estimator: GeomagneticAttitudeEstimator,
            attitude: Quaternion,
            roll: Double?,
            pitch: Double?,
            yaw: Double?,
            coordinateTransformation: CoordinateTransformation?
        )
    }
}