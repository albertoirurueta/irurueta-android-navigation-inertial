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

package com.irurueta.android.navigation.inertial.calibration.noise

/**
 * Mode to use to stop the an estimator.
 */
enum class StopMode {
    /**
     * Takes into account maximum number of samples to be processed only.
     */
    MAX_SAMPLES_ONLY,

    /**
     * Takes into account maximum duration to take measurements only.
     */
    MAX_DURATION_ONLY,

    /**
     * Takes into account maximum number of samples to be processed or maximum duration,
     * whichever comes first.
     */
    MAX_SAMPLES_OR_DURATION
}