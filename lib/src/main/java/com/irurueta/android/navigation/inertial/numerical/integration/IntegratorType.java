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
 * Indicates type of integrator.
 */
public enum IntegratorType {
    /**
     * Quadrature integrator. Suitable for general purpose integrations when no prior knowledge
     * of the integrand is known or the function is not very smooth (i.e. linearly interpolated).
     * Has slow convergence and might not converge at all if required accuracy is too stringent.
     */
    QUADRATURE,

    /**
     * Simpson integrator. Suitable when assumptions can be made about integrand smoothness.
     * In general Simpson method will be more efficient (i.e. requires fewer function evaluations)
     * when the function to be integrated has a finite fourth derivative (i.e. a continuous third
     * derivative).
     */
    SIMPSON,

    /**
     * Romberg integrator. Suitable when integrand is sufficiently smooth (e.g. analytic), and
     * integrated over intervals that contain no singularities, and where the endpoints are also
     * non-singular. In such circumstances, Romberg method is more efficient than Simpson's one.
     */
    ROMBERG
}
