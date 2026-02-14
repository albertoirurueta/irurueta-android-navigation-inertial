package com.irurueta.android.navigation.inertial.old.processors.attitude

import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.old.collectors.AccelerometerAndMagnetometerSyncedSensorMeasurement

/**
 * Estimates leveled absolute attitude using accelerometer and magnetometer sensors.
 * Roll and pitch Euler angles are leveled using accelerometer sensor.
 * Yaw angle is obtained from magnetometer once the leveling is estimated.
 *
 * @property processorListener listener to notify new leveled absolute attitudes.
 */
class AccelerometerGeomagneticAttitudeProcessor(
    processorListener: OnProcessedListener<AccelerometerSensorMeasurement,
            AccelerometerAndMagnetometerSyncedSensorMeasurement>? = null
) : BaseGeomagneticAttitudeProcessor<AccelerometerSensorMeasurement,
        AccelerometerAndMagnetometerSyncedSensorMeasurement>(processorListener) {

    /**
     * Internal processor to estimate gravity from accelerometer sensor measurements.
     */
    override val gravityProcessor = AccelerometerGravityProcessor()

    /**
     * Processes provided synced measurement to estimate fused leveled absolute attitude.
     *
     * @param syncedMeasurement synced measurement to be processed.
     * @return true if a new leveled absolute attitude is processed, false otherwise.
     */
    override fun process(
        syncedMeasurement: AccelerometerAndMagnetometerSyncedSensorMeasurement
    ): Boolean {
        val accelerometerMeasurement = syncedMeasurement.accelerometerMeasurement
        val magnetometerMeasurement = syncedMeasurement.magnetometerMeasurement
        return if (accelerometerMeasurement != null && magnetometerMeasurement != null) {
            process(accelerometerMeasurement, magnetometerMeasurement)
        } else {
            false
        }
    }
}