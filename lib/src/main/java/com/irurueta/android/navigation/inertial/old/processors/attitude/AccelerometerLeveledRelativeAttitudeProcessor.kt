package com.irurueta.android.navigation.inertial.old.processors.attitude

import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.old.collectors.AccelerometerAndGyroscopeSyncedSensorMeasurement

/**
 * Estimates leveled relative attitude by fusing leveling attitude obtained
 * from accelerometer sensor, and relative attitude obtained from gyroscope sensor.
 *
 * @property processorListener listener to notify new leveled relative attitudes.
 */
class AccelerometerLeveledRelativeAttitudeProcessor(
    processorListener: OnProcessedListener<AccelerometerSensorMeasurement,
            AccelerometerAndGyroscopeSyncedSensorMeasurement>? = null
) : BaseLeveledRelativeAttitudeProcessor<AccelerometerSensorMeasurement,
        AccelerometerAndGyroscopeSyncedSensorMeasurement>(processorListener) {

    /**
     * Internal processor to estimate gravity from accelerometer sensor measurements.
     */
    override val gravityProcessor = AccelerometerGravityProcessor()

    /**
     * Processes provided synced measurement to estimate current leveled relative attitude.
     *
     * @param syncedMeasurement synced measurement to be processed.
     * @return true if a new leveled relative attitude has been estimated, false otherwise.
     */
    override fun process(syncedMeasurement: AccelerometerAndGyroscopeSyncedSensorMeasurement): Boolean {
        val accelerometerMeasurement = syncedMeasurement.accelerometerMeasurement
        val gyroscopeMeasurement = syncedMeasurement.gyroscopeMeasurement
        return if (accelerometerMeasurement != null && gyroscopeMeasurement != null) {
            process(accelerometerMeasurement, gyroscopeMeasurement, syncedMeasurement.timestamp)
        } else {
            false
        }
    }
}