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
package com.irurueta.android.navigation.inertial.old.collectors

/**
 * Bases class for synced sensor measurements, which contains 2 oor more [com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement]
 * instances that have been synced and assumed to belong to the same timestamp.
 *
 * @property timestamp timestamp expressed in nanoseconds following
 * [android.os.SystemClock.elapsedRealtimeNanos] monotonic clock when [com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement] are
 * assumed to occur.
 */
abstract class SyncedSensorMeasurement(var timestamp: Long)