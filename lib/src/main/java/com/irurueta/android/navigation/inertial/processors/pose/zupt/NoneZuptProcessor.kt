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

package com.irurueta.android.navigation.inertial.processors.pose.zupt

import com.irurueta.android.navigation.inertial.collectors.measurements.SyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.WithAccelerometerSensorMeasurement

/**
 * ZUPT processor that assumes that NO ZUPT is ever made
 */
class NoneZuptProcessor<T> :
    ZuptProcessor<T>() where T : SyncedSensorMeasurement<T>, T : WithAccelerometerSensorMeasurement {

    /**
     * Processes synced measurement to determine whether ZUPT (Zero Velocity Update) should be
     * made or not.
     * This implementation assumes that NO ZUPT is ever made, consequently, always returns 0.0.
     *
     * @return A value between 0.0 and 1.0 indicating whether ZUPT should be made or not. 0.0
     * indicates that ZUPT should not be made, while 1.0 indicates that ZUPT should be made.
     */
    override fun process(syncedMeasurement: T): Double {
        return 0.0
    }

    /**
     * Evaluates accelerometer average magnitude and variance to determine whether ZUPT (Zero
     * Velocity Update) conditions apply or not.
     * This implementation assumes that NO ZUPT is ever made, consequently, always returns 0.0.
     *
     * @return 1.0 if ZUPT conditions apply, 0.0 otherwise.
     *
     */
    override fun evaluateAccelerometer(): Double {
        return 0.0
    }

    /**
     * Evaluates accelerometer average magnitude and variance, and gyroscope variance to
     * determine whether ZUPT (Zero Velocity Update) conditions apply or not.
     * This implementation assumes that NO ZUPT is ever made, consequently, always returns 0.0.
     *
     * @return 1.0 if ZUPT conditions apply, 0.0 otherwise.
     */
    override fun evaluateAccelerometerAndGyroscope(): Double {
        return 0.0
    }
}