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
package com.irurueta.android.navigation.inertial.old.processors.pose

import android.location.Location
import com.irurueta.android.navigation.inertial.ENUtoNEDConverter
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.geometry.EuclideanTransformation3D
import com.irurueta.geometry.InhomogeneousPoint3D
import com.irurueta.geometry.Quaternion
import com.irurueta.geometry.Rotation3D
import com.irurueta.navigation.frames.*
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.navigators.ECEFInertialNavigator
import com.irurueta.units.TimeConverter

/**
 * Base class to estimate absolute pose expressed in ECEF coordinates.
 * Implementations of this subclass might use a combination of: accelerometer, gyroscope,
 * magnetometer, gravity or attitude measurements.
 *
 * @property initialLocation initial device location.
 * @property initialVelocity initial velocity of device expressed in NED coordinates.
 * @property estimatePoseTransformation true to estimate 3D metric pose transformation.
 * @property processorListener listener to notify new poses.
 */
abstract class BaseECEFAbsolutePoseProcessor(
    val initialLocation: Location,
    val initialVelocity: NEDVelocity,
    val estimatePoseTransformation: Boolean,
    var processorListener: OnProcessedListener?
) {
    /**
     * Device frame expressed in ECEF coordinates when estimator starts.
     * This is reused for performance reasons.
     */
    val initialEcefFrame = ECEFFrame()

    /**
     * Device frame expressed in NED coordinates when estimator starts.
     * This is reused for performance reasons.
     */
    val initialNedFrame = NEDFrame()

    /**
     * Device frame expressed in ECEF coordinates for previous set of measurements.
     * This is reused for performance reasons.
     */
    val previousEcefFrame = ECEFFrame()

    /**
     * Device frame expressed in NED coordinates for previous set of measurements.
     * This is reused for performance reasons.
     */
    val previousNedFrame = NEDFrame()

    /**
     * Current device frame expressed in ECEF coordinates.
     * This is reused for performance reasons.
     */
    val currentEcefFrame = ECEFFrame()

    /**
     * Current device frame expressed in NED coordinates.
     * This is reused for performance reasons.
     */
    val currentNedFrame = NEDFrame()

    /**
     * 3D metric pose transformation.
     * This is reused for performance reasons.
     */
    val poseTransformation = EuclideanTransformation3D()

    /**
     * True indicates that attitude is leveled and expressed respect to estimator start.
     * False indicates that attitude is absolute.
     */
    var useLeveledRelativeAttitudeRespectStart = true

    /**
     * Time interval expressed in seconds between consecutive gyroscope measurements
     */
    var timeIntervalSeconds = 0.0
        private set

    /**
     * Attitude of current sample.
     */
    protected val currentAttitude = Quaternion()

    /**
     * Initial attitude when estimator starts. This is used to obtain a relative leveled pose
     * respect to the initial attitude.
     */
    private val initialAttitude = Quaternion()

    /**
     * Body kinematics containing las accelerometer and gyroscope measurements.
     * This is reused for performance reasons.
     */
    private val bodyKinematics = BodyKinematics()

    /**
     * Specific force measured by accelerometer sensor containing both device acceleration and
     * gravity component expressed in NED coordinates.
     */
    private val specificForce = AccelerationTriad()

    /**
     * Angular speed measured by gyroscope sensor.
     */
    private val angularSpeed = AngularSpeedTriad()

    /**
     * True indicates that estimator has been initialized with an initial frame,
     * containing estimated absolute attitude and current device location (if available).
     */
    private var initializedFrame = false

    /**
     * Array containing euler angles. This is reused for performance reasons during pose
     * transformation computation.
     */
    private val eulerAngles = DoubleArray(Rotation3D.INHOM_COORDS)

    /**
     * Contains pose transformation attitude. This is reused for performance reasons.
     */
    private val transformationRotation = Quaternion()

    /**
     * Contains position variation respect to initial position expressed in ECEF coordinates.
     * This is reused for performance reasons during pose transformation computation.
     */
    private val ecefDiffPosition = InhomogeneousPoint3D()

    /**
     * Contains initial attitude in ECEF coordinates system. This is reused for performance reasons
     * during pose transformation computation.
     */
    private val startEcefRotation = Quaternion()

    /**
     * Contains inverse of initial attitude in ECEF coordinates system.
     * This is used to obtain local coordinates for translation during pose transformation
     * computation.
     */
    private val inverseEcefRotation = Quaternion()

    /**
     * Contains position variation expressed in local coordinates.
     * This is reused for performance reasons during pose transformation computation.
     */
    private val localDiffPosition = InhomogeneousPoint3D()

    /**
     * Timestamp of previous sample expressed in nanoseconds.
     */
    private var previousTimestamp: Long = -1L

    /**
     * Instance to be reused containing coordinate transformation in NED coordinates.
     */
    private val coordinateTransformation =
        CoordinateTransformation(FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME)

    /**
     * Resets internal parameters.
     */
    open fun reset() {
        initializedFrame = false
        previousTimestamp = -1L
        timeIntervalSeconds = 0.0
    }

    /**
     * Processes pose by taking into account estimated current attitude, accelerometer and
     * gyroscope measurements to estimate new position and velocity from previous ones.
     *
     * @param accelerometerMeasurement accelerometer measurement.
     * @param gyroscopeMeasurement gyroscope measurement.
     * @param timestamp timestamp when all measurements are assumed to occur.
     * @return true if new pose is processed, false otherwise.
     */
    protected open fun processPose(
        accelerometerMeasurement: AccelerometerSensorMeasurement,
        gyroscopeMeasurement: GyroscopeSensorMeasurement,
        timestamp: Long
    ): Boolean {
        if (!processTimeInterval(timestamp)) {
            return false
        }

        initializeFrameIfNeeded(currentAttitude)

        processAccelerometer(accelerometerMeasurement)
        processGyroscope(gyroscopeMeasurement)

        // compensate measured gravity respect theoretical gravity
        bodyKinematics.fx = specificForce.valueX
        bodyKinematics.fy = specificForce.valueY
        bodyKinematics.fz = specificForce.valueZ

        // compensate measured angular speed respect theoretical Earth rotation
        bodyKinematics.angularRateX = angularSpeed.valueX
        bodyKinematics.angularRateY = angularSpeed.valueY
        bodyKinematics.angularRateZ = angularSpeed.valueZ

        ECEFInertialNavigator.navigateECEF(
            timeIntervalSeconds,
            previousEcefFrame,
            bodyKinematics,
            currentEcefFrame
        )

        // Ensure that attitude does not drift by resetting current local attitude in NED
        // coordinates and converting back to ECEF coordinates
        ECEFtoNEDFrameConverter.convertECEFtoNED(currentEcefFrame, currentNedFrame)
        currentNedFrame.coordinateTransformationRotation = currentAttitude
        NEDtoECEFFrameConverter.convertNEDtoECEF(currentNedFrame, currentEcefFrame)

        // compute transformation between initial and current frame and previous and current frame
        if (estimatePoseTransformation) {
            computeTransformation(
                initialEcefFrame,
                currentEcefFrame,
                initialAttitude,
                currentAttitude,
                poseTransformation
            )
        }

        // notify
        val poseTransformation =
            if (estimatePoseTransformation) poseTransformation else null

        processorListener?.onProcessed(
            this,
            currentEcefFrame,
            previousEcefFrame,
            initialEcefFrame,
            timestamp,
            poseTransformation
        )

        // update previous frame
        previousEcefFrame.copyFrom(currentEcefFrame)
        previousNedFrame.copyFrom(currentNedFrame)

        return true
    }

    /**
     * Processes current accelerometer measurement to obtain specific force expressed in NED
     * coordinates.
     *
     * @param measurement accelerometer measurement to be processed.
     */
    private fun processAccelerometer(measurement: AccelerometerSensorMeasurement) {
        val ax = measurement.ax.toDouble()
        val ay = measurement.ay.toDouble()
        val az = measurement.az.toDouble()
        val bx = measurement.bx?.toDouble()
        val by = measurement.by?.toDouble()
        val bz = measurement.bz?.toDouble()

        val currentAx = if (bx != null) ax - bx else ax
        val currentAy = if (by != null) ay - by else ay
        val currentAz = if (bz != null) az - bz else az

        ENUtoNEDConverter.convert(currentAx, currentAy, currentAz, specificForce)
    }

    /**
     * Processes current gyroscope measurement to obtain angular speed expressed in NED
     * coordinates.
     *
     * @param measurement gyroscope measurement to be processed.
     */
    private fun processGyroscope(measurement: GyroscopeSensorMeasurement) {
        val wx = measurement.wx.toDouble()
        val wy = measurement.wy.toDouble()
        val wz = measurement.wz.toDouble()
        val bx = measurement.bx?.toDouble()
        val by = measurement.by?.toDouble()
        val bz = measurement.bz?.toDouble()

        val currentWx = if (bx != null) wx - bx else wx
        val currentWy = if (by != null) wy - by else wy
        val currentWz = if (bz != null) wz - bz else wz

        ENUtoNEDConverter.convert(currentWx, currentWy, currentWz, angularSpeed)
    }

    /**
     * Updates current time interval estimation between gyroscope measurements.
     *
     * @param timestamp optional timestamp that can be provided to override timestamp associated to
     * gyroscope measurement. If null, the timestamp from gyroscope measurement is used.
     * @return true if it is NOT the 1st measurement and time interval has been estimated, false
     * otherwise.
     */
    private fun processTimeInterval(timestamp: Long): Boolean {
        val isNotFirst = previousTimestamp > 0
        if (isNotFirst) {
            val diff = timestamp - previousTimestamp
            timeIntervalSeconds = TimeConverter.nanosecondToSecond(diff.toDouble())
        }
        previousTimestamp = timestamp

        return isNotFirst
    }

    /**
     * Estimates initial ECEF frame when the first absolute attitude is obtained.
     *
     * @param attitude first obtained attitude.
     */
    private fun initializeFrameIfNeeded(
        attitude: Quaternion,
    ) {
        if (!initializedFrame) {
            coordinateTransformation.fromRotation(attitude)

            initialNedFrame.coordinateTransformation = coordinateTransformation
            initialNedFrame.position = initialLocation.toNEDPosition()
            initialNedFrame.velocity = initialVelocity

            NEDtoECEFFrameConverter.convertNEDtoECEF(initialNedFrame, initialEcefFrame)

            initialEcefFrame.copyTo(previousEcefFrame)

            initialAttitude.fromQuaternion(attitude)

            initializedFrame = true
        }
    }

    /**
     * Computes 3D metric transformation between provided ECEF frames and local attitudes.
     *
     * @param initialEcefFrame starting frame.
     * @param currentEcefFrame end frame.
     * @param initialAttitude start body attitude respect to local coordinates.
     * @param currentAttitude end body attitude respect to local coordinates.
     * @param result instance where result will be stored.
     */
    private fun computeTransformation(
        initialEcefFrame: ECEFFrame,
        currentEcefFrame: ECEFFrame,
        initialAttitude: Quaternion,
        currentAttitude: Quaternion,
        result: EuclideanTransformation3D,
    ) {
        // set rotation
        if (useLeveledRelativeAttitudeRespectStart) {
            initialAttitude.toEulerAngles(eulerAngles)
            val initYaw = eulerAngles[2]

            currentAttitude.toEulerAngles(eulerAngles)
            val currentRoll = eulerAngles[0]
            val currentPitch = eulerAngles[1]
            val currentYaw = eulerAngles[2]

            val deltaYaw = currentYaw - initYaw
            transformationRotation.setFromEulerAngles(currentRoll, currentPitch, deltaYaw)

            // convert to ENU coordinates
            ENUtoNEDConverter.convert(transformationRotation, transformationRotation)
        } else {
            // convert to ENU coordinates
            ENUtoNEDConverter.convert(currentAttitude, transformationRotation)
        }

        result.rotation.fromRotation(transformationRotation)

        // current attitude contains body to local frame attitude.
        // ECEF frame contains euclidean position coordinates rotated by body rotation respect ECEF.
        // we need to transform position coordinates respect ECEF to euclidean coordinates respect
        // local position
        ecefDiffPosition.setInhomogeneousCoordinates(
            currentEcefFrame.x - initialEcefFrame.x,
            currentEcefFrame.y - initialEcefFrame.y,
            currentEcefFrame.z - initialEcefFrame.z
        )

        initialEcefFrame.coordinateTransformation.asRotation(startEcefRotation)
        startEcefRotation.inverseRotation(inverseEcefRotation)

        inverseEcefRotation.rotate(ecefDiffPosition, localDiffPosition)

        // convert from NED to ENU
        ENUtoNEDConverter.convertPoint(localDiffPosition, localDiffPosition)

        result.setTranslation(localDiffPosition)
    }

    /**
     * Interface to notify when a new pose has been processed.
     */
    fun interface OnProcessedListener {
        /**
         * Called whe a new pose is processed.
         *
         * @param processor processor that raised this event.
         * @param currentFrame current ECEF frame containing device position, velocity and attitude.
         * @param previousFrame ECEF frame of previous measurement.
         * @param initialFrame initial ECEF frame when processor was started.
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * will be monotonically increasing using the same time base as
         * [android.os.SystemClock.elapsedRealtimeNanos].
         * @param poseTransformation 3D metric transformation containing leveled attitude and
         * translation variation since this processor started expressed in ENU system of
         * coordinates.
         */
        fun onProcessed(
            processor: BaseECEFAbsolutePoseProcessor,
            currentFrame: ECEFFrame,
            previousFrame: ECEFFrame,
            initialFrame: ECEFFrame,
            timestamp: Long,
            poseTransformation: EuclideanTransformation3D?
        )
    }
}