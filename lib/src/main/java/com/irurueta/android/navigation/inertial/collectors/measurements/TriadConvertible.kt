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

package com.irurueta.android.navigation.inertial.collectors.measurements

import com.irurueta.navigation.inertial.calibration.Triad

/**
 * Indicates that implementing classes can be converted to a [Triad] instance.
 * This is useful when dealing with sensor measurements that can be represented as a triad of
 * values, such as accelerometer, gyroscope, or magnetometer measurements.
 */
interface TriadConvertible<T : Triad<*, *>> {

    /**
     * Converts the implementing class to a [Triad] instance and stores the result in the provided
     * [result] instance.
     *
     * @param result instance where the converted triad will be stored.
     */
    fun toTriad(result: T)

    /**
     * Converts the implementing class to a new [Triad] instance.
     *
     * @return a new [Triad] instance representing the converted values.
     */
    fun toTriad(): T
}