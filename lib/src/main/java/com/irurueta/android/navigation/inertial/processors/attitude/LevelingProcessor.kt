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

package com.irurueta.android.navigation.inertial.processors.attitude

import com.irurueta.navigation.inertial.estimators.LevelingEstimator

/**
 * Estimates leveling of device (roll and pitch angle) in NED coordinates by using estimated gravity
 * vector.
 * This processor does not estimate attitude yaw angle, as either a magnetometer or gyroscope would
 * be needed.
 *
 * @property processorListener listener to notify new leveled attitudes.
 */
class LevelingProcessor(processorListener: OnProcessedListener? = null) :
    BaseLevelingProcessor(processorListener) {

    /**
     * Processes provided gravity components estimated using a [BaseGravityProcessor].
     *
     * @param gx x-coordinate of sensed specific force containing gravity component expressed in
     * NED coordinates and meters per squared second (m/s^2).
     * @param gy y-coordinate of sensed specific force containing gravity component expressed in
     * NED coordinates and meters per squared second (m/s^2).
     * @param gz z-coordinate of sensed specific force containing gravity component expressed in
     * NED coordinates and meters per squared second (m/s^2).
     */
    override fun process(gx: Double, gy: Double, gz: Double) {
        val roll = LevelingEstimator.getRoll(gy, gz)
        val pitch = LevelingEstimator.getPitch(
            gx,
            gy,
            gz
        )

        attitude.setFromEulerAngles(roll, pitch, 0.0)

        this.gx = gx
        this.gy = gy
        this.gz = gz

        processorListener?.onProcessed(this, attitude)
    }
}