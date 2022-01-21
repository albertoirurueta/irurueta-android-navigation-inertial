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
package com.irurueta.android.navigation.inertial

import android.location.Location
import com.irurueta.navigation.frames.ECEFPosition
import com.irurueta.navigation.frames.ECEFVelocity
import com.irurueta.navigation.frames.NEDPosition
import com.irurueta.navigation.frames.NEDVelocity
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter
import com.irurueta.navigation.inertial.ECEFGravity
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator

/**
 * Utility class to obtain gravity at a given location.
 */
object GravityHelper {

    /**
     * Gets gravity at provided location.
     *
     * @param location Earth location where gravity needs to be found.
     * @return expected gravity.
     */
    fun getGravityForLocation(location: Location): ECEFGravity {
        return getGravityForPosition(location.toNEDPosition())
    }

    /**
     * Gets gravity at provided NED position.
     *
     * @param nedPosition Earth position expressed in NED coordinates.
     * @return expected gravity.
     */
    fun getGravityForPosition(nedPosition: NEDPosition): ECEFGravity {
        val nedVelocity = NEDVelocity()
        val ecefPosition = ECEFPosition()
        val ecefVelocity = ECEFVelocity()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            nedPosition,
            nedVelocity,
            ecefPosition,
            ecefVelocity
        )
        return ECEFGravityEstimator.estimateGravityAndReturnNew(
            ecefPosition.x,
            ecefPosition.y,
            ecefPosition.z
        )
    }

    /**
     * Gets gravity norm at provided location expressed in meters per squared second (m/s^2).
     *
     * @param location Earth location where gravity needs to be found.
     * @return expected gravity norm.
     */
    fun getGravityNormForLocation(location: Location): Double {
        return getGravityForLocation(location).norm
    }
}