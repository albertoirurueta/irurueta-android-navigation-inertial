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
package com.irurueta.android.navigation.inertial.old.collectors

import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement

/**
 * Contains synced attitude, accelerometer and gyroscope measurements, which are assumed to belong
 * to the same timestamp.
 *
 * @property attitudeMeasurement an attitude measurement (either relative or absolute). Notice that
 * this instance might be reused between consecutive
 * [AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement] measurements, and that its timestamp
 * might differ from the global [timestamp] property of this [SyncedSensorMeasurement]
 * @property accelerometerMeasurement an accelerometer measurement. Notice that this instance might
 * be reused between consecutive [AccelerometerAndGyroscopeSyncedSensorMeasurement] measurements,
 * and that its timestamp might differ from the global [timestamp] property of this
 * [SyncedSensorMeasurement].
 * @property gyroscopeMeasurement a gyroscope measurement. Notice that this instance might be reused
 * between consecutive [AccelerometerAndGyroscopeSyncedSensorMeasurement] measurements, and that its
 * timestamp might differ from the global [timestamp] property of this [SyncedSensorMeasurement].
 * @property timestamp timestamp expressed in nanoseconds following
 * [android.os.SystemClock.elapsedRealtimeNanos] monotonic clock when synced [com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement] are
 * assumed to occur.
 */
class AttitudeAccelerometerAndGyroscopeSyncedSensorMeasurement(
    var attitudeMeasurement: AttitudeSensorMeasurement? = null,
    var accelerometerMeasurement: AccelerometerSensorMeasurement? = null,
    var gyroscopeMeasurement: GyroscopeSensorMeasurement? = null,
    timestamp: Long = 0L
) : SyncedSensorMeasurement(timestamp)