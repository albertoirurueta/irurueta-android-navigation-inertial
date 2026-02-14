package com.irurueta.android.navigation.inertial.old.processors.attitude

import com.irurueta.android.navigation.inertial.ENUtoNEDConverter
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.geometry.Quaternion

/**
 * Processes an absolute or relative attitude obtained by an Android Sensor expressed in ENU
 * coordinates system and converts it to NED coordinates system.
 *
 * @property processorListener listener to notify new attitudes.
 */
class AttitudeProcessor(var processorListener: OnProcessedListener? = null) {

    /**
     * Converted absolute or relative attitude in NED coordinates.
     */
    var nedAttitude = Quaternion()
        private set

    /**
     * Processes an attitude measurement containing an absolute or relative attitude obtained by one
     * of the available android sensors expressed in ENU coordinates system.
     *
     * @param measurement measurement to be converted.
     * @return converted attitude in NED coordinates.
     */
    fun process(measurement: AttitudeSensorMeasurement): Quaternion {
        val enuAttitude = measurement.attitude

        ENUtoNEDConverter.convert(enuAttitude, nedAttitude)

        processorListener?.onProcessed(
            this,
            nedAttitude,
            measurement.timestamp,
            measurement.accuracy
        )

        return nedAttitude
    }

    /**
     * Interface to notify when a new attitude measurement has been processed.
     */
    fun interface OnProcessedListener {
        /**
         * Called when a new attitude measurement is processed.
         *
         * @param processor processor that raised this event.
         * @param attitude containing absolute or relative device attitude expressed in NED
         * coordinates.
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * will be monotonically increasing using the same time base as
         * [android.os.SystemClock.elapsedRealtimeNanos].
         * @param accuracy attitude sensor accuracy.
         */
        fun onProcessed(
            processor: AttitudeProcessor,
            attitude: Quaternion,
            timestamp: Long,
            accuracy: SensorAccuracy?
        )
    }
}