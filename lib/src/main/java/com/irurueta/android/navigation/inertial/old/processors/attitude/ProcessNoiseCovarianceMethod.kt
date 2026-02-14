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
package com.irurueta.android.navigation.inertial.old.processors.attitude

/**
 * Supported methods to estimate process noise covariance matrix
 */
enum class ProcessNoiseCovarianceMethod {
    /**
     * Precise method. Requires more computational power.
     */
    PRECISE,

    /**
     * Better approximation method
     */
    BETTER,

    /**
     * Approximated method. Assumes short time intervals between samples to approximate
     * computation of matrix integrals.
     */
    APPROXIMATED
}