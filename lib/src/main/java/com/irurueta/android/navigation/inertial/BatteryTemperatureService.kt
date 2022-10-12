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

import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.os.BatteryManager
import com.irurueta.units.Temperature
import com.irurueta.units.TemperatureUnit

/**
 * Service to obtain battery temperature.
 *
 * @property context Android contxt.
 * @property batteryTemperatureChangedListener listener to be notified each time that battery
 * temperature changes.
 */
class BatteryTemperatureService(
    val context: Context,
    var batteryTemperatureChangedListener: OnBatteryTemperatureChangedListener? = null
) {
    /**
     * Receiver to be notified each time that battery temperature changes.
     */
    private val receiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context?, intent: Intent?) {
            if (intent != null) {
                val temperature = intent.getIntExtra(BatteryManager.EXTRA_TEMPERATURE, 0)
                val temperatureCelsius = temperature.toFloat() / 10.0f
                batteryTemperatureChangedListener?.onBatteryTemperatureChanged(temperatureCelsius)
            }
        }
    }

    /**
     * Obtains current battery temperature expressed in Celsius.
     */
    val currentBatteryTemperatureCelsius: Float?
        get() {
            val status = context.registerReceiver(null, IntentFilter(Intent.ACTION_BATTERY_CHANGED))
            return if (status != null) {
                val temperature = status.getIntExtra(BatteryManager.EXTRA_TEMPERATURE, 0)
                temperature.toFloat() / 10.0f
            } else {
                null
            }
        }

    /**
     * Obtains current battery temperature.
     */
    val currentBatteryTemperature: Temperature?
        get() {
            val value = currentBatteryTemperatureCelsius
            return if (value != null) {
                Temperature(value, TemperatureUnit.CELSIUS)
            } else {
                null
            }
        }

    /**
     * Obtains current battery temperature.
     *
     * @param result instance where result will be stored.
     * @return true if result was stored, false if battery temperature cannot be determined.
     */
    fun getCurrentBatteryTemperature(result: Temperature): Boolean {
        val value = currentBatteryTemperatureCelsius
        return if (value != null) {
            result.value = value
            result.unit = TemperatureUnit.CELSIUS
            true
        } else {
            false
        }
    }

    /**
     * Starts notifying current battery temperature each time that temperature changes.
     */
    fun start() {
        context.registerReceiver(receiver, IntentFilter(Intent.ACTION_BATTERY_CHANGED))
    }

    /**
     * Stops notifying current battery temperature each time that temperature changes.
     */
    fun stop() {
        context.unregisterReceiver(receiver)
    }

    /**
     * Interface to notify when battery temperature changes.
     */
    fun interface OnBatteryTemperatureChangedListener {
        /**
         * Notifies that battery temperature has changed.
         *
         * @param temperatureCelsius current battery temperature expressed in Celsius.
         */
        fun onBatteryTemperatureChanged(temperatureCelsius: Float)
    }
}