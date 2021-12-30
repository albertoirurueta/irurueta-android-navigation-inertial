/*
 * Copyright (C) 2021 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial

import android.location.Location
import com.irurueta.navigation.frames.NEDPosition

/**
 * Converts an Android [Location] into a [NEDPosition].
 *
 * @param result instance where result will be stored.
 */
fun Location.toNEDPosition(result: NEDPosition) {
    result.latitude = Math.toRadians(this.latitude)
    result.longitude = Math.toRadians(this.longitude)
    result.height = this.altitude
}

/**
 * Converts an Android [Location] into a [NEDPosition].
 *
 * @return converted [NEDPosition].
 */
fun Location.toNEDPosition(): NEDPosition {
    val result = NEDPosition()
    toNEDPosition(result)
    return result
}