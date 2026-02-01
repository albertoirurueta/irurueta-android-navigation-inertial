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

package com.irurueta.android.navigation.inertial.calibration.intervals

import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import org.junit.Assert
import org.junit.Test

class StatusTest {

    @Test
    fun mapStatus_returnsExpectedValue() {
        Assert.assertEquals(
            Status.FAILED,
            Status.mapStatus(TriadStaticIntervalDetector.Status.IDLE, true)
        )
        Assert.assertEquals(
            Status.FAILED,
            Status.mapStatus(TriadStaticIntervalDetector.Status.INITIALIZING, true)
        )
        Assert.assertEquals(
            Status.FAILED,
            Status.mapStatus(TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED, true)
        )
        Assert.assertEquals(
            Status.FAILED,
            Status.mapStatus(TriadStaticIntervalDetector.Status.STATIC_INTERVAL, true)
        )
        Assert.assertEquals(
            Status.FAILED,
            Status.mapStatus(TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, true)
        )
        Assert.assertEquals(
            Status.FAILED,
            Status.mapStatus(TriadStaticIntervalDetector.Status.FAILED, true)
        )
        Assert.assertEquals(
            Status.IDLE,
            Status.mapStatus(TriadStaticIntervalDetector.Status.IDLE, false)
        )
        Assert.assertEquals(
            Status.INITIALIZING,
            Status.mapStatus(TriadStaticIntervalDetector.Status.INITIALIZING, false)
        )
        Assert.assertEquals(
            Status.INITIALIZATION_COMPLETED,
            Status.mapStatus(TriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED, false)
        )
        Assert.assertEquals(
            Status.STATIC_INTERVAL,
            Status.mapStatus(TriadStaticIntervalDetector.Status.STATIC_INTERVAL, false)
        )
        Assert.assertEquals(
            Status.DYNAMIC_INTERVAL,
            Status.mapStatus(TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, false)
        )
        Assert.assertEquals(
            Status.FAILED,
            Status.mapStatus(TriadStaticIntervalDetector.Status.FAILED, false)
        )
    }
}