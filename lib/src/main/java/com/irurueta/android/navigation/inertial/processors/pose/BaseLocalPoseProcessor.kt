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

package com.irurueta.android.navigation.inertial.processors.pose

import android.location.Location
import com.irurueta.algebra.Matrix
import com.irurueta.algebra.Utils
import com.irurueta.android.navigation.inertial.ENUtoNEDConverter
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.geometry.EuclideanTransformation3D
import com.irurueta.geometry.InhomogeneousPoint3D
import com.irurueta.geometry.MatrixRotation3D
import com.irurueta.geometry.Quaternion
import com.irurueta.geometry.Rotation3D
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.ECEFFrame
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.frames.NEDFrame
import com.irurueta.navigation.frames.NEDVelocity
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter
import com.irurueta.navigation.inertial.NEDGravity
import com.irurueta.navigation.inertial.RadiiOfCurvature
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.calibration.Triad
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator
import com.irurueta.navigation.inertial.estimators.RadiiOfCurvatureEstimator
import com.irurueta.navigation.inertial.navigators.NEDInertialNavigator
import com.irurueta.units.TimeConverter
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan

/**
 * Base class to estimate absolute pose using local plane navigation.
 * Implementations of this subclass might use a combination of: accelerometer, gyroscope,
 * magnetometer, gravity or attitude measurements.
 *
 * @property initialLocation initial device location.
 * @property initialVelocity initial velocity of device expressed in NED coordinates.
 * @property estimatePoseTransformation true to estimate 3D metric pose transformation.
 * @property processorListener listener to notify new poses.
 */
abstract class BaseLocalPoseProcessor(
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
     * Accelerometer measurement in NED coordinates.
     */
    private val nedAccelerometerMeasurement = AccelerometerSensorMeasurement()

    /**
     * Specific force measured by accelerometer sensor containing both device acceleration and
     * gravity component expressed in NED coordinates.
     */
    private val specificForce = AccelerationTriad()

    /**
     * Gyroscope measurement in NED coordinates.
     */
    private val nedGyroscopeMeasurement = GyroscopeSensorMeasurement()

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
     * Attitude of previous sample.
     */
    private val previousAttitude = Quaternion()

    /**
     * Attitude of current sample.
     */
    protected val currentAttitude = Quaternion()

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
     * Initial attitude when estimator starts. This is used to obtain a relative leveled pose
     * respect to the initial attitude.
     */
    private val initialAttitude = Quaternion()

    /**
     * Average attitude between current and previous attitudes.
     */
    private val averageAttitude = Quaternion()

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
     * Contains reused specific force expressed in body coordinates
     */
    private val fibb = Matrix(Triad.COMPONENTS, 1)

    /**
     * Contains average body attitude rotation matrix to convert from body to NED local
     * coordinates.
     */
    private val averageAttitudeMatrix = Matrix(
        MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
        MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS
    )

    /**
     * Contains reused specific force expressed in NED local coordinates.
     */
    private val fibn = Matrix(Triad.COMPONENTS, 1)

    /**
     * Contains gravity expressed in NED local coordinates at current location.
     * Because Earth is not completely spherical, gravity does not only point downwards, there is
     * also a small north component.
     */
    private val gravity = NEDGravity()

    /**
     * Contains gravity expressed in NED local coordinates at current location and in matrix form.
     * Because Earth is not completely spherical, gravity does not only point downwards, there is
     * also a small north component.
     */
    private val g = Matrix(NEDGravity.COMPONENTS, 1)

    /**
     * Contains previous velocity of body frame expressed in NED coordinates.
     */
    private val oldVelocity = NEDVelocity()

    /**
     * Contains previous velocity of body frame expressed in ECEF coordinates and in matrix form.
     */
    private val oldVebn = Matrix(ROWS, 1)

    /**
     * Contains angular rate of Earth rotation NED local coordinates.
     */
    private val omegaIen = Matrix(ROWS, 1)

    /**
     * Radii of curvature of the WGS84 Earth ellipsoid at previous location latitude.
     */
    private val oldRadiiOfCurvature = RadiiOfCurvature()

    /**
     * Contains Earth rotation component based on current body speed in NED local coordinates
     */
    private val oldOmegaEnN = Matrix(ROWS, 1)

    /**
     * Contains total Earth rotation based on the sum of Earth rotation and body speed.
     */
    private val oldOmegaN = Matrix(ROWS, 1)

    /**
     * Skew matrix containing total Earth rotation
     */
    private val skewOmega2 = Matrix(ROWS, ROWS)

    /**
     * Body acceleration expressed in NED local coordinates (without Coriolis force)
     */
    private val abn = Matrix(Triad.COMPONENTS, 1)

    /**
     * Acceleration caused by Earth rotation (Coriolis force)
     */
    private val aOmegaN = Matrix(Triad.COMPONENTS, 1)

    /**
     * Body acceleration expressed in NED local coordinates (taking into account Coriolis force)
     */
    private val abn2 = Matrix(Triad.COMPONENTS, 1)

    /**
     * Velocity variation of body frame expressed in ECEF coordinates and in matrix form.
     */
    private val deltaVebn = Matrix(ROWS, 1)

    /**
     * Current velocity of body frame expressed in ECEF coordinates and in matrix form.
     */
    private val vEbn = Matrix(ROWS, 1)

    /**
     * Radii of curvature of the WGS84 Earth ellipsoid at current location latitude.
     */
    private val radiiOfCurvature = RadiiOfCurvature()

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
        // NOTE: References in comments refer to formulas in Paul D. Groves. Principles of GNSS,
        // Intertial and multisensor integrated navigation systems
        if (!processTimeInterval(timestamp)) {
            return false
        }

        initializeFrameIfNeeded(currentAttitude)

        processAccelerometer(accelerometerMeasurement)
        processGyroscope(gyroscopeMeasurement)

        // obtain average attitude between current and previous attitude
        Quaternion.slerp(previousAttitude, currentAttitude, 0.5, averageAttitude)
        averageAttitude.normalize()

        // Transform specific force to NED frame
        specificForce.getValuesAsMatrix(fibb)

        averageAttitude.asInhomogeneousMatrix(averageAttitudeMatrix)
        averageAttitudeMatrix.multiply(fibb, fibn)

        NEDGravityEstimator.estimateGravity(currentNedFrame, gravity)
        gravity.asMatrix(g)

        previousNedFrame.getVelocity(oldVelocity)
        val oldVn = oldVelocity.vn
        val oldVe = oldVelocity.ve
        val oldVd = oldVelocity.vd
        oldVebn.setElementAtIndex(0, oldVn)
        oldVebn.setElementAtIndex(1, oldVe)
        oldVebn.setElementAtIndex(2, oldVd)

        // From (2.123), determine the angular rate of the ECEF frame with respect
        // the ECI frame, resolved about NED
        val oldLatitude = previousNedFrame.latitude
        omegaIen.setElementAtIndex(
            0,
            cos(oldLatitude) * NEDInertialNavigator.EARTH_ROTATION_RATE
        )
        omegaIen.setElementAtIndex(1, 0.0)
        omegaIen.setElementAtIndex(
            2,
            -sin(oldLatitude) * NEDInertialNavigator.EARTH_ROTATION_RATE
        )

        // From (5.44), determine the angular rate of the NED frame with respect
        // the ECEF frame, resolved about NED
        RadiiOfCurvatureEstimator.estimateRadiiOfCurvature(oldLatitude, oldRadiiOfCurvature)
        val oldRe = oldRadiiOfCurvature.re
        val oldRn = oldRadiiOfCurvature.rn

        val oldHeight = previousNedFrame.height

        oldOmegaEnN.setElementAtIndex(0, oldVe / (oldRe + oldHeight))
        oldOmegaEnN.setElementAtIndex(1, -oldVn / (oldRn + oldHeight))
        oldOmegaEnN.setElementAtIndex(2, -oldVe * tan(oldLatitude) / (oldRe + oldHeight))

        omegaIen.multiplyByScalar(2.0)

        oldOmegaEnN.add(omegaIen, oldOmegaN)

        Utils.skewMatrix(oldOmegaN, skewOmega2)

        // Update velocity
        // From (5.54),
        fibn.add(g, abn)

        skewOmega2.multiply(oldVebn, aOmegaN)
        abn.subtract(aOmegaN, abn2)

        abn2.copyTo(deltaVebn)
        deltaVebn.multiplyByScalar(timeIntervalSeconds)

        oldVebn.add(deltaVebn, vEbn)

        val vn = vEbn.getElementAtIndex(0)
        val ve = vEbn.getElementAtIndex(1)
        val vd = vEbn.getElementAtIndex(2)

        // Update curvilinear position
        // Update height using (5.56)
        val height: Double = oldHeight - 0.5 * timeIntervalSeconds * (oldVd + vd)

        // Update latitude using (5.56)
        val latitude: Double = (oldLatitude
                + 0.5 * timeIntervalSeconds * (oldVn / (oldRn + oldHeight) + vn / (oldRn + height)))

        // Calculate meridian and transverse radii of curvature
        RadiiOfCurvatureEstimator.estimateRadiiOfCurvature(latitude, radiiOfCurvature)
        val re = radiiOfCurvature.re

        // Update longitude using (5.56)
        val oldLongitude = previousNedFrame.longitude
        val longitude: Double = oldLongitude +
                0.5 * timeIntervalSeconds * (oldVe / ((oldRe + oldHeight) * cos(oldLatitude)) +
                ve / ((re + height) * cos(latitude)))

        currentNedFrame.setPosition(latitude, longitude, height)
        currentNedFrame.setVelocityCoordinates(vn, ve, vd)
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
        currentAttitude.copyTo(previousAttitude)

        return true
    }

    /**
     * Processes current accelerometer measurement to obtain specific force expressed in NED
     * coordinates.
     *
     * @param measurement accelerometer measurement to be processed.
     */
    private fun processAccelerometer(measurement: AccelerometerSensorMeasurement) {
        measurement.toNed(nedAccelerometerMeasurement)
        nedAccelerometerMeasurement.toTriad(specificForce)
    }

    /**
     * Processes current gyroscope measurement to obtain angular speed expressed in NED
     * coordinates.
     *
     * @param measurement gyroscope measurement to be processed.
     */
    private fun processGyroscope(measurement: GyroscopeSensorMeasurement) {
        measurement.toNed(nedGyroscopeMeasurement)
        nedGyroscopeMeasurement.toTriad(angularSpeed)
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
    private fun initializeFrameIfNeeded(attitude: Quaternion) {
        if (!initializedFrame) {
            coordinateTransformation.fromRotation(attitude)

            initialNedFrame.coordinateTransformation = coordinateTransformation
            initialNedFrame.position = initialLocation.toNEDPosition()
            initialNedFrame.velocity = initialVelocity

            NEDtoECEFFrameConverter.convertNEDtoECEF(initialNedFrame, initialEcefFrame)

            initialEcefFrame.copyTo(previousEcefFrame)
            initialNedFrame.copyTo(previousNedFrame)

            initialAttitude.fromQuaternion(attitude)
            previousAttitude.fromQuaternion(attitude)

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

    private companion object {

        /**
         * Number of rows in a matrix representation of triads.
         */
        const val ROWS = 3
    }

    /**
     * Interface to notify when a new pose has been processed.
     */
    fun interface OnProcessedListener {
        /**
         * Called when a new pose is processed.
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
            processor: BaseLocalPoseProcessor,
            currentFrame: ECEFFrame,
            previousFrame: ECEFFrame,
            initialFrame: ECEFFrame,
            timestamp: Long,
            poseTransformation: EuclideanTransformation3D?
        )
    }
}