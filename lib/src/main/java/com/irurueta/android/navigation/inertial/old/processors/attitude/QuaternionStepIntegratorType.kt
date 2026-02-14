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
 * Indicates type of quaternion integrator step. Different types exist with different levels of
 * accuracy and computation accuracy.
 */
enum class QuaternionStepIntegratorType {
    /**
     * Performs quaternion integration using Suh's method, which has a medium accuracy and
     * computational complexity.
     */
    SUH,

    /**
     * Performs quaternion integration using Trawny's method, which has a medium accuracy and
     * computational complexity.
     */
    TRAWNY,

    /**
     * Performs quaternion integration using Yuan's meth0d, which has a medium accuracy and
     * computational complexity.
     */
    YUAN,

    /**
     * Performs quaternion integration using Euler's method, which is the item accurate and has
     * the smallest computational complexity.
     */
    EULER_METHOD,

    /**
     * Performs quaternion integration using mid-point algorithm, which offers a medium accuracy
     * an computational complexity.
     */
    MID_POINT,

    /**
     * Performs quaternion integration using Runge-Kutta of 4th order (aka RK4) algorithm, which
     * offers high accuracy at the expense of higher computational complexity.
     */
    RUNGE_KUTTA
}