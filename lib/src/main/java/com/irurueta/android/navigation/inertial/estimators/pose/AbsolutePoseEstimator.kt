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
package com.irurueta.android.navigation.inertial.estimators.pose

import android.location.Location
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorCollector
import com.irurueta.geometry.EuclideanTransformation3D
import com.irurueta.navigation.frames.ECEFFrame
import com.irurueta.navigation.frames.NEDVelocity
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import java.util.*

/**
 * Base interface for absolute pose estimators where absolute device position and attitude
 * respect to Earth is known.
 */
interface AbsolutePoseEstimator : PoseEstimator {

    /**
     * True to estimate 3D metric transformation, false to skip
     * transformation computation.
     */
    val estimatePoseTransformation: Boolean

    /**
     * One of the supported magnetometer sensor types.
     */
    val magnetometerSensorType: MagnetometerSensorCollector.SensorType

    /**
     * Initial velocity of device expressed in NED coordinates.
     */
    val initialVelocity: NEDVelocity

    /**
     * Listener to notify new magnetometer measurements.
     */
    var magnetometerMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener?

    /**
     * Gets or sets initial device location.
     */
    var initialLocation: Location

    /**
     * Earth's magnetic model. If null, the default model is used if [useWorldMagneticModel] is
     * true. If [useWorldMagneticModel] is false, this is ignored.
     */
    var worldMagneticModel: WorldMagneticModel?

    /**
     * Timestamp when World Magnetic Model will be evaluated to obtain current.
     * Only taken into account if [useWorldMagneticModel] is tue.
     */
    var timestamp: Date

    /**
     * Indicates whether world magnetic model is taken into account to adjust attitude yaw angle by
     * current magnetic declination based on current World Magnetic Model, location and timestamp.
     */
    var useWorldMagneticModel: Boolean

    /**
     * True indicates that attitude is leveled and expressed respect to estimator start.
     * False indicates that attitude is absolute.
     */
    val useLeveledRelativeAttitudeRespectStart: Boolean

    /**
     * Interface to notify when a new pose is available.
     */
    fun interface OnPoseAvailableListener<T: AbsolutePoseEstimator> {

        /**
         * Called when a new pose is available.
         *
         * @param estimator pose estimator that raised this event.
         * @param currentFrame current ECEF frame containing device position, velocity and attitude.
         * @param previousFrame ECEF frame of previous measurement.
         * @param initialFrame initial ECEF frame when estimator was started.
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * wil be monotonically increasing using the same time base as
         * [android.os.SystemClock.elapsedRealtimeNanos].
         * @param poseTransformation 3D metric transformation containing leveled attitude and
         * translation variation since this estimator started expressed in ENU system of
         * coordinates.
         */
        fun onPoseAvailable(
            estimator: T,
            currentFrame: ECEFFrame,
            previousFrame: ECEFFrame,
            initialFrame: ECEFFrame,
            timestamp: Long,
            poseTransformation: EuclideanTransformation3D?
        )
    }
}