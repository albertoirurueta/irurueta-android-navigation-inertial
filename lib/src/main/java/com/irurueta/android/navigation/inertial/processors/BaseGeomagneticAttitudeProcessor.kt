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
package com.irurueta.android.navigation.inertial.processors

import android.location.Location
import com.irurueta.android.navigation.inertial.ENUtoNEDTriadConverter
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.SyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.NEDPosition
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.navigation.inertial.estimators.AttitudeEstimator
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.units.MagneticFluxDensityConverter
import java.util.*

/**
 * Base class to estimate leveled absolute attitude using accelerometer (or gravity) and
 * magnetometer sensors.
 * Roll and pitch Euler angles are leveled using accelerometer or gravity sensors.
 * Yaw angle is obtained from magnetometer once the leveling is estimated.
 *
 * @property processorListener listener to notify new leveled absolute attitudes.
 *
 * @param M type of accelerometer or gravity sensor measurement.
 * @param S type of synced sensor measurement.
 */
abstract class BaseGeomagneticAttitudeProcessor<M : SensorMeasurement<M>, S : SyncedSensorMeasurement>(
    var processorListener: OnProcessedListener<M, S>?
) {
    /**
     * Internal processor to estimate gravity from accelerometer or gravity sensor measurements.
     */
    protected abstract val gravityProcessor: BaseGravityProcessor<M>

    /**
     * Internal processor to estimate leveling attitude from estimated gravity.
     */
    private lateinit var levelingProcessor: BaseLevelingProcessor

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
     * Triad to be reused for ENU to NED coordinates conversion.
     */
    private val triad = MagneticFluxDensityTriad()

    /**
     * Array to be reused containing euler angles of leveling attitude.
     */
    private val eulerAngles = DoubleArray(Quaternion.N_ANGLES)

    /**
     * Instance to be reused containing estimated attitude in NED coordinates by merging leveled
     * attitude with magnetometer measurements.
     */
    val fusedAttitude = Quaternion()

    /**
     * X-coordinates of last sensed gravity component of specific force expressed in NED coordinates
     * and in meters per squared second (m/s^2).
     */
    val gx: Double
        get() = gravityProcessor.gx

    /**
     * Y-coordinate of last sensed gravity component of specific force expressed in NED coordinates
     * and in meters per squared second (m/s^2).
     */
    val gy: Double
        get() = gravityProcessor.gy

    /**
     * Z-coordinate of last sensed gravity component of specific force expressed in NED coordinates
     * and in meters per squared second (m/s^2).
     */
    val gz: Double
        get() = gravityProcessor.gz

    /**
     * Gets a new triad containing gravity component of specific force expressed in NED coordinates
     * and in meters per squared second (m/s^2).
     */
    val gravity: AccelerationTriad
        get() = gravityProcessor.gravity

    /**
     * Updates provided triad to contain gravity component of specific force expressed in NED
     * coordinates and in meters per squared second (m/s^2).
     */
    fun getGravity(result: AccelerationTriad) {
        gravityProcessor.getGravity(result)
    }

    /**
     * Gets or sets device location
     */
    var location: Location? = null

    /**
     * Timestamp being used when World Magnetic Model is evaluated to obtain current magnetic
     * declination. This is only taken into account if [useWorldMagneticModel] is true.
     * If not defined, current date is assumed.
     */
    var currentDate: Date? = null

    /**
     * Indicates whether accurate leveling must be used or not.
     *
     * @throws IllegalStateException if set to true and no location is available.
     */
    var useAccurateLevelingProcessor: Boolean = false
        @Throws(IllegalStateException::class)
        set(value) {
            if (value) {
                checkNotNull(location)
            }

            field = value
            buildLevelingProcessor()
        }

    /**
     * Earth's magnetic model. If null, the default model is used if [useWorldMagneticModel] is
     * true. If [useWorldMagneticModel] is false, this is ignored.
     */
    var worldMagneticModel: WorldMagneticModel? = null
        set(value) {
            field = value
            buildWMMEstimator()
        }

    /**
     * Indicates whether world magnetic model is taken into account to adjust attitude yaw angle by
     * current magnetic declination based on current World Magnetic Model, location and timestamp.
     */
    var useWorldMagneticModel: Boolean = false
        set(value) {
            field = value
            buildWMMEstimator()
        }

    /**
     * Processes provided synced measurement to estimate fused leveled absolute attitude.
     *
     * @param syncedMeasurement synced measurement to be processed.
     * @return true if a new leveled absolute attitude is processed, false otherwise.
     */
    abstract fun process(syncedMeasurement: S): Boolean

    /**
     * Resets internal parameters.
     */
    fun reset() {
        gravityProcessor.reset()
        levelingProcessor.reset()

        fusedAttitude.setFromEulerAngles(0.0, 0.0, 0.0)
    }

    /**
     * Processes provided measurements to estimate fused leveled absolute attitude.
     *
     * @param accelerometerOrGravityMeasurement accelerometer or gravity measurement.
     * @param magnetometerMeasurement magnetometer measurement.
     * @return true if new leveled absolute attitude is processed, false otherwise.
     */
    internal fun process(
        accelerometerOrGravityMeasurement: M,
        magnetometerMeasurement: MagnetometerSensorMeasurement,
    ): Boolean {
        return if (gravityProcessor.process(accelerometerOrGravityMeasurement)) {
            val gx = gravityProcessor.gx
            val gy = gravityProcessor.gy
            val gz = gravityProcessor.gz
            levelingProcessor.process(gx, gy, gz)

            // convert magnetometer measurement to NED coordinates
            processMagnetometerMeasurement(magnetometerMeasurement)

            // process leveling with current leveling attitude and converted magnetometer
            // measurement
            processLeveling(levelingProcessor.attitude)

            // notify
            processorListener?.onProcessed(
                this,
                fusedAttitude,
                accelerometerOrGravityMeasurement.accuracy,
                magnetometerMeasurement.accuracy
            )

            true
        } else {
            false
        }
    }

    /**
     * Processes magnetometer measurement and converts it to NED coordinates.
     *
     * @param measurement magnetometer measurement.
     */
    private fun processMagnetometerMeasurement(measurement: MagnetometerSensorMeasurement) {
        val bx = measurement.bx
        val by = measurement.by
        val bz = measurement.bz
        val hardIronX = measurement.hardIronX
        val hardIronY = measurement.hardIronY
        val hardIronZ = measurement.hardIronZ

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
    }

    /**
     * Processes last received leveling attitude to obtain an updated absolute attitude taking
     * into account last magnetometer measurement.
     *
     * @param attitude estimated absolute attitude respect NED coordinate system.
     */
    private fun processLeveling(attitude: Quaternion) {
        attitude.copyTo(levelingAttitude)

        // obtain roll and pitch euler angles from leveling
        levelingAttitude.toEulerAngles(eulerAngles)
        val roll = eulerAngles[0]
        val pitch = eulerAngles[1]
        val declination = getDeclination()
        val yaw = AttitudeEstimator.getYaw(
            triad.valueX,
            triad.valueY,
            triad.valueZ,
            declination,
            roll,
            pitch
        )

        fusedAttitude.setFromEulerAngles(roll, pitch, yaw)
    }

    /**
     * Obtains magnetic declination expressed in radians if World Magnetic Model is used,
     * otherwise zero is returned.
     *
     * @return magnetic declination.
     */
    private fun getDeclination(): Double {
        val wmmEstimator = this.wmmEstimator ?: return 0.0
        val location = this.location ?: return 0.0

        location.toNEDPosition(position)
        return wmmEstimator.getDeclination(
            position.latitude, position.longitude, position.height, currentDate ?: Date()
        )
    }

    /**
     * Builds the internal leveling processor.
     */
    private fun buildLevelingProcessor() {
        levelingProcessor = if (useAccurateLevelingProcessor) {
            val location = this.location
            checkNotNull(location)

            AccurateLevelingProcessor(location)
        } else {
            LevelingProcessor()
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

    init {
        // initializes this processor
        buildLevelingProcessor()
    }

    /**
     * Interface to notify when a new leveled absolute attitude has been processed.
     *
     * @param M type of accelerometer or gravity sensor measurement.
     * @param S type of synced sensor measurement.
     */
    fun interface OnProcessedListener<M : SensorMeasurement<M>, S : SyncedSensorMeasurement> {
        fun onProcessed(
            processor: BaseGeomagneticAttitudeProcessor<M, S>,
            fusedAttitude: Quaternion,
            accelerometerOrGravityAccuracy: SensorAccuracy?,
            magnetometerAccuracy: SensorAccuracy?
        )
    }
}