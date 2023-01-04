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

import com.irurueta.geometry.Quaternion

/**
 * Base class to estimate leveling of device (roll and pitch angle) by using estimated gravity
 * vector.
 *
 * @property processorListener listener to notify new gravity measurements.
 */
abstract class BaseLevelingProcessor(var processorListener: OnProcessedListener?) {

    /**
     *
     * Instance to be reused containing estimated leveling attitude (roll and pitch angles) in NED
     * coordinates.
     */
    var attitude = Quaternion()
        protected set

    /**
     * Processes provided gravity components estimated using a [BaseGravityProcessor].
     */
    abstract fun process(gx: Double, gy: Double, gz: Double)

    /**
     * Resets this processor to its initial values.
     */
    fun reset() {
        attitude.setFromEulerAngles(0.0, 0.0, 0.0)
    }

    /**
     * Interface to notify when a new leveled attitude has been processed.
     */
    fun interface OnProcessedListener {
        /**
         * Called when a new leveled attitude measurement is processed.
         *
         * @param processor processor that raised this event.
         * @param attitude estimated leveled attitude.
         */
        fun onProcessed(
            processor: BaseLevelingProcessor,
            attitude: Quaternion
        )
    }
}