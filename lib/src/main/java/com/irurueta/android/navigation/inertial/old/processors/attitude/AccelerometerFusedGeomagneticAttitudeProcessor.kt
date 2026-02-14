package com.irurueta.android.navigation.inertial.old.processors.attitude

import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.old.collectors.AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement

/**
 * Estimates absolute attitude by fusing absolute leveled attitude and relative attitude.
 *
 * @property processorListener listener to notify new fused absolute attitudes.
 */
class AccelerometerFusedGeomagneticAttitudeProcessor(
    processorListener: OnProcessedListener<AccelerometerSensorMeasurement, AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement>? = null
) : BaseFusedGeomagneticAttitudeProcessor<AccelerometerSensorMeasurement, AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement>(
    processorListener
) {

    /**
     * Internal processor to estimate leveled absolute attitude.
     */
    override val geomagneticProcessor = AccelerometerGeomagneticAttitudeProcessor()

    /**
     * Processes provided synced measurement to estimate current fused absolute attitude.
     *
     * @param syncedMeasurement synced measurement to be processed.
     * @return true if a new fused absolute attitude has been estimated, false otherwise.
     */
    override fun process(syncedMeasurement: AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement): Boolean {
        val accelerometerMeasurement = syncedMeasurement.accelerometerMeasurement
        val gyroscopeMeasurement = syncedMeasurement.gyroscopeMeasurement
        val magnetometerMeasurement = syncedMeasurement.magnetometerMeasurement
        return if (accelerometerMeasurement != null && gyroscopeMeasurement != null && magnetometerMeasurement != null) {
            process(accelerometerMeasurement, gyroscopeMeasurement, magnetometerMeasurement)
        } else {
            false
        }
    }
}