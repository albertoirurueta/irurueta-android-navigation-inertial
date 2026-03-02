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
 * Builder for ZUPT processors.
 *
 * @param settings settings for ZUPT processor creation.
 */
class ZuptProcessorBuilder<T>(val settings: ZuptSettings = ZuptSettings())
        where T : SyncedSensorMeasurement<T>, T : WithAccelerometerSensorMeasurement {

    /**
     * Builds a new instance of a ZuptProcessor.
     *
     * @return a new instance of a ZuptProcessor.
     */
    fun build(): ZuptProcessor<T> {
        return when (settings.processorType) {
            ZuptProcessorType.NONE -> NoneZuptProcessor()
            ZuptProcessorType.NON_ADAPTIVE -> NonAdaptiveZuptProcessor(
                settings.location,
                settings.gravityThreshold,
                settings.accelerometerVarianceThreshold,
                settings.gyroscopeVarianceThreshold,
                settings.windowNanoseconds
            )

            ZuptProcessorType.ADAPTIVE -> AdaptiveZuptProcessor(
                settings.location,
                settings.gravityThreshold,
                settings.accelerometerNoiseVariance,
                settings.gyroscopeNoiseVariance,
                settings.accelerometerFactor,
                settings.gyroscopeFactor,
                settings.windowNanoseconds
            )

            ZuptProcessorType.SOFT -> SoftZuptProcessor(
                settings.location,
                settings.gravityWeight,
                settings.accelerometerWeight,
                settings.gyroscopeWeight,
                settings.gravityNormalizationFactor,
                settings.accelerometerNormalizationFactor,
                settings.gyroscopeNormalizationFactor,
                settings.windowNanoseconds
            )
        }
    }
}