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
package com.irurueta.android.navigation.inertial.calibration

import com.irurueta.android.navigation.inertial.calibration.intervals.ErrorReason
import io.mockk.clearAllMocks
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Test

class CalibratorErrorReasonTest {

    @After
    fun tearDown() {
        clearAllMocks()
    }

    @Test
    fun mapErrorReason_returnsExpectedValue() {
        assertEquals(
            CalibratorErrorReason.UNRELIABLE_SENSOR,
            CalibratorErrorReason.mapErrorReason(ErrorReason.UNRELIABLE_SENSOR)
        )
        assertEquals(
            CalibratorErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION,
            CalibratorErrorReason.mapErrorReason(ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION)
        )
        assertEquals(
            CalibratorErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION,
            CalibratorErrorReason.mapErrorReason(ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED_DURING_INITIALIZATION)
        )
    }
}