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
package com.irurueta.android.navigation.inertial

import android.content.Context
import android.os.Build
import android.view.Surface
import android.view.WindowManager

/**
 * Helper class to obtain UI display orientation respect to the device screen.
 */
object DisplayOrientationHelper {

    /**
     * Gets UI display orientation respect to the device screen.
     *
     * @return orientation expressed in degrees. Can be either 0.0, 90.0, 180.0 or 270.0 degrees.
     */
    @Suppress("DEPRECATION")
    fun getDisplayRotationDegrees(context: Context): Double {
        // Device orientation (portrait, landscape, etc)
        val rotation =  if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
            context.display?.rotation
        } else {
            val windowManager = context.getSystemService(
                Context.WINDOW_SERVICE
            ) as WindowManager

            windowManager.defaultDisplay.rotation
        }

        return when (rotation) {
            Surface.ROTATION_0 -> 0.0
            Surface.ROTATION_90 -> 90.0
            Surface.ROTATION_180 -> 180.0
            Surface.ROTATION_270 -> 270.0
            else -> 0.0
        }
    }

    /**
     * Gets UI display orientation respect to the device screen.
     *
     * @return orientation expressed in radians.
     * @see [getDisplayRotationDegrees]
     */
    fun getDisplayRotationRadians(context: Context): Double {
        return Math.toRadians(getDisplayRotationDegrees(context))
    }
}