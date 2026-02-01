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

package com.irurueta.android.navigation.inertial.calibration.intervals

import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector

/**
 * Interval detection status values.
 */
enum class Status {
    /**
     * Detector is in idle status when it hasn't processed any sample yet.
     */
    IDLE,

    /**
     * Detector is processing samples in the initial static interval to determine base noise
     * level.
     */
    INITIALIZING,

    /**
     * Detector has successfully completed processing samples on the initial static period.
     */
    INITIALIZATION_COMPLETED,

    /**
     * A static interval has been detected, where sensor is considered to be subject to
     * no substantial movement forces.
     */
    STATIC_INTERVAL,

    /**
     * A dynamic interval has been detected, where sensor is considered to be subject to
     * substantial movement forces.
     */
    DYNAMIC_INTERVAL,

    /**
     * Detector has failed. This happens if sensor is subject to sudden movement forces
     * while detector is initializing during the initial static period, if there is too much
     * overall motion during initialization, or if sensor becomes unreliable.
     * When detector has failed, no new samples will be allowed to be processed until detector
     * is reset.
     */
    FAILED;

    companion object {
        /**
         * Maps a [com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector.Status] into a [Status].
         *
         * @param status status to map from.
         * @param unreliable when true, mapping result is always [Status.FAILED].
         * @return mapped status.
         */
        fun mapStatus(status: TriadStaticIntervalDetector.Status?, unreliable: Boolean): Status {
            return if (unreliable) {
                FAILED
            } else {
                when (status) {
                    TriadStaticIntervalDetector.Status.IDLE -> IDLE
                    TriadStaticIntervalDetector.Status.INITIALIZING -> INITIALIZING
                    TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED -> INITIALIZATION_COMPLETED
                    TriadStaticIntervalDetector.Status.STATIC_INTERVAL -> STATIC_INTERVAL
                    TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL -> DYNAMIC_INTERVAL
                    TriadStaticIntervalDetector.Status.FAILED -> FAILED
                    else -> IDLE
                }
            }
        }
    }
}