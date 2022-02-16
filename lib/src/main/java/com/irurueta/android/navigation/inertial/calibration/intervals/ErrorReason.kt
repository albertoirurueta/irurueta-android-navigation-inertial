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
package com.irurueta.android.navigation.inertial.calibration.intervals

import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector

/**
 * Reason why interval detection fails.
 */
enum class ErrorReason {
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
    UNRELIABLE_SENSOR;

    companion object {
        /**
         * Maps a [TriadStaticIntervalDetector.ErrorReason] into an [ErrorReason].
         *
         * @param reason reason to map from.
         * @param unreliable when true, mapping result is always [ErrorReason.UNRELIABLE_SENSOR].
         * @return mapped reason.
         */
        fun mapErrorReason(
            reason: TriadStaticIntervalDetector.ErrorReason,
            unreliable: Boolean
        ): ErrorReason {
            return if (unreliable) {
                UNRELIABLE_SENSOR
            } else {
                return when (reason) {
                    TriadStaticIntervalDetector.ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED ->
                        OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
                    TriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED ->
                        SUDDEN_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION
                }
            }
        }
    }
}