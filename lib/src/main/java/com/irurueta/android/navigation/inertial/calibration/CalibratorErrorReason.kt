/*
 * Copyright (C) 2025 Alberto Irurueta Carro (alberto@irurueta.com)
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

package com.irurueta.android.navigation.inertial.calibration

import com.irurueta.android.navigation.inertial.calibration.intervals.ErrorReason

/**
 * Reasons why a calibrator can fail.
 */
enum class CalibratorErrorReason {
    /**
     * If a sudden movement is detected during initialization.
     */
    SUDDEN_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION,

    /**
     * If overall noise level is excessive during initialization.
     */
    OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION,

    /**
     * If sensor becomes unreliable.
     */
    UNRELIABLE_SENSOR,

    /**
     * Occurs if obtained measurements cannot yield a numerically stable solution
     * during calibration estimation.
     */
    NUMERICAL_INSTABILITY_DURING_CALIBRATION;

    companion object {
        /**
         * Maps interval detector error reasons into error reasons returned by a calibrator.
         *
         * @param reason reason to map from.
         * @return mapped reason.
         */
        fun mapErrorReason(reason: ErrorReason): CalibratorErrorReason {
            return when (reason) {
                ErrorReason.UNRELIABLE_SENSOR
                -> UNRELIABLE_SENSOR
                ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
                -> SUDDEN_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
                ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
                -> OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
            }
        }
    }
}