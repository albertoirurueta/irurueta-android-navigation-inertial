package com.irurueta.android.navigation.inertial.old.processors.attitude

import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.old.collectors.AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement

/**
 * Estimates absolute attitude by fusing absolute leveled geomagnetic attitude and leveled relative
 * attitude.
 *
 * @property processorListener listener to notify new fused absolute attitude.
 */
class AccelerometerDoubleFusedGeomagneticAttitudeProcessor(processorListener: OnProcessedListener<AccelerometerSensorMeasurement, AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement>? = null
) : BaseDoubleFusedGeomagneticAttitudeProcessor<AccelerometerSensorMeasurement, AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement>(processorListener) {

    /**
     * Internal processor to estimate leveled absolute attitude.
     */
    override val geomagneticProcessor = AccelerometerGeomagneticAttitudeProcessor()

    /**
     * Internal processor to estimate leveled relative attitude.
     */
    override val relativeGyroscopeProcessor = AccelerometerLeveledRelativeAttitudeProcessor()

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