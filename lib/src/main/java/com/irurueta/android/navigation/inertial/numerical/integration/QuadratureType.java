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
package com.irurueta.android.navigation.inertial.numerical.integration;

/**
 * Indicates type of quadrature.
 */
public enum QuadratureType {
    /**
     * Trapezoidal quadrature. It's the simplest for general purpose integration.
     */
    TRAPEZOIDAL,

    /**
     * Mid point quadrature. Allows singularities at integration bounds.
     */
    MID_POINT,

    /**
     * Infinity mid point. Allows upper bound to be infinity, or lower bound negative and infinity.
     */
    INFINITY_MID_POINT,

    /**
     * Lower square root mid point. Allows a singularity of the integrand at the lower integration
     * bound.
     */
    LOWER_SQUARE_ROOT_MID_POINT,

    /**
     * Upper square root mid point. Allows a singularity of the integrand at the upper integration
     * bound.
     */
    UPPER_SQUARE_ROOT_MID_POINT,

    /**
     * Exponential mid point quadrature. Allows integration from a lower bound up to infinity when
     * the integrand falls off exponentially.
     */
    EXPONENTIAL_MID_POINT,

    /**
     * Double exponential rule. Allows integration when there are singularities at integration
     * bounds with fast convergence.
     */
    DOUBLE_EXPONENTIAL_RULE
}
