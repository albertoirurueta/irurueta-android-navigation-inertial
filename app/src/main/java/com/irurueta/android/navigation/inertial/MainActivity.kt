package com.irurueta.android.navigation.inertial

import android.content.Intent
import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.widget.AppCompatButton
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector

class MainActivity : AppCompatActivity() {
    private var absoluteAttitudeButton: AppCompatButton? = null

    private var relativeAttitudeButton: AppCompatButton? = null

    private var geomagneticAttitudeButton: AppCompatButton? = null

    private var levelingAttitudeButton: AppCompatButton? = null

    private var improvedLevelingAttitudeButton: AppCompatButton? = null

    private var geomagneticLevelingAttitudeButton: AppCompatButton? = null

    private var gravityLevelingButton: AppCompatButton? = null

    private var accelerometerLowPassFilterLevelingButton: AppCompatButton? = null

    private var accelerometerMeanFilterLevelingButton: AppCompatButton? = null

    private var accelerometerMedianFilterLevelingButton: AppCompatButton? = null

    private var accelerometerUncalibratedLowPassFilterLevelingButton: AppCompatButton? = null

    private var accelerometerUncalibratedMeanFilterLevelingButton: AppCompatButton? = null

    private var accelerometerUncalibratedMedianFilterLevelingButton: AppCompatButton? = null

    private var gravityAccurateLevelingButton: AppCompatButton? = null

    private var accelerometerLowPassFilterAccurateLevelingButton: AppCompatButton? = null

    private var accelerometerMeanFilterAccurateLevelingButton: AppCompatButton? = null

    private var accelerometerMedianFilterAccurateLevelingButton: AppCompatButton? = null

    private var accelerometerUncalibratedLowPassFilterAccurateLevelingButton: AppCompatButton? =
        null

    private var accelerometerUncalibratedMeanFilterAccurateLevelingButton: AppCompatButton? = null

    private var accelerometerUncalibratedMedianFilterAccurateLevelingButton: AppCompatButton? = null

    private var relativeGyroscopeAttitudeButton: AppCompatButton? = null

    private var relativeUncalibratedGyroscopeAttitudeButton: AppCompatButton? = null

    private var accurateRelativeGyroscopeAttitudeButton: AppCompatButton? = null

    private var accurateRelativeUncalibratedGyroscopeAttitudeButton: AppCompatButton? = null

    private var leveledRelativeAttitudeEstimatorGravityLevelingButton: AppCompatButton? = null

    private var leveledRelativeAttitudeEstimatorAccelerometerLowPassFilterLevelingButton: AppCompatButton? =
        null

    private var leveledRelativeAttitudeEstimatorAccelerometerMeanFilterLevelingButton: AppCompatButton? =
        null

    private var leveledRelativeAttitudeEstimatorAccelerometerMedianFilterLevelingButton: AppCompatButton? =
        null

    private var leveledRelativeAttitudeEstimatorAccelerometerUncalibratedLowPassFilterLevelingButton: AppCompatButton? =
        null

    private var leveledRelativeAttitudeEstimatorAccelerometerUncalibratedMeanFilterLevelingButton: AppCompatButton? =
        null

    private var leveledRelativeAttitudeEstimatorAccelerometerUncalibratedMedianFilterLevelingButton: AppCompatButton? =
        null

    private var leveledRelativeAttitudeEstimatorUncalibratedGyroscopeButton: AppCompatButton? = null

    private var leveledRelativeAttitudeEstimatorGravityAccurateLevelingButton: AppCompatButton? = null

    private var leveledRelativeAttitudeEstimatorAccelerometerLowPassFilterAccurateLevelingButton: AppCompatButton? =
        null

    private var leveledRelativeAttitudeEstimatorAccelerometerMeanFilterAccurateLevelingButton: AppCompatButton? =
        null

    private var leveledRelativeAttitudeEstimatorAccelerometerMedianFilterAccurateLevelingButton: AppCompatButton? =
        null

    private var leveledRelativeAttitudeEstimatorAccelerometerUncalibratedLowPassFilterAccurateLevelingButton: AppCompatButton? =
        null

    private var leveledRelativeAttitudeEstimatorAccelerometerUncalibratedMeanFilterAccurateLevelingButton: AppCompatButton? =
        null

    private var leveledRelativeAttitudeEstimatorAccelerometerUncalibratedMedianFilterAccurateLevelingButton: AppCompatButton? =
        null

    private var leveledRelativeAttitudeEstimatorUncalibratedGyroscopeAccurateLevelingButton: AppCompatButton? = null

    private var leveledRelativeAttitudeEstimatorGravityAccurateAttitudeButton: AppCompatButton? = null

    private var leveledRelativeAttitudeEstimatorAccelerometerLowPassFilterAccurateAttitudeButton: AppCompatButton? =
        null

    private var leveledRelativeAttitudeEstimatorAccelerometerMeanFilterAccurateAttitudeButton: AppCompatButton? =
        null

    private var leveledRelativeAttitudeEstimatorAccelerometerMedianFilterAccurateAttitudeButton: AppCompatButton? =
        null

    private var leveledRelativeAttitudeEstimatorAccelerometerUncalibratedLowPassFilterAccurateAttitudeButton: AppCompatButton? =
        null

    private var leveledRelativeAttitudeEstimatorAccelerometerUncalibratedMeanFilterAccurateAttitudeButton: AppCompatButton? =
        null

    private var leveledRelativeAttitudeEstimatorAccelerometerUncalibratedMedianFilterAccurateAttitudeButton: AppCompatButton? =
        null

    private var leveledRelativeAttitudeEstimatorUncalibratedGyroscopeAccurateAttitudeButton: AppCompatButton? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        absoluteAttitudeButton = findViewById(R.id.absolute_attitude_button)
        absoluteAttitudeButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, AttitudeSensorCollectorActivity::class.java)
            intent.putExtra(
                AttitudeSensorCollectorActivity.SENSOR_TYPE,
                AttitudeSensorCollectorActivity.ABSOLUTE_ATTITUDE
            )
            startActivity(intent)
        }

        relativeAttitudeButton = findViewById(R.id.relative_attitude_button)
        relativeAttitudeButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, AttitudeSensorCollectorActivity::class.java)
            intent.putExtra(
                AttitudeSensorCollectorActivity.SENSOR_TYPE,
                AttitudeSensorCollectorActivity.RELATIVE_ATTITUDE
            )
            startActivity(intent)
        }

        geomagneticAttitudeButton = findViewById(R.id.geomagnetic_attitude_button)
        geomagneticAttitudeButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, AttitudeSensorCollectorActivity::class.java)
            intent.putExtra(
                AttitudeSensorCollectorActivity.SENSOR_TYPE,
                AttitudeSensorCollectorActivity.GEOMAGNETIC_ABSOLUTE_ATTITUDE
            )
            startActivity(intent)
        }

        levelingAttitudeButton = findViewById(R.id.leveling_attitude_button)
        levelingAttitudeButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, AttitudeEstimatorActivity::class.java)
            intent.putExtra(
                AttitudeEstimatorActivity.ESTIMATOR_TYPE,
                AttitudeEstimatorActivity.LEVELING
            )
            startActivity(intent)
        }

        improvedLevelingAttitudeButton = findViewById(R.id.improved_leveling_attitude_button)
        improvedLevelingAttitudeButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, AttitudeEstimatorActivity::class.java)
            intent.putExtra(
                AttitudeEstimatorActivity.ESTIMATOR_TYPE,
                AttitudeEstimatorActivity.IMPROVED_LEVELING
            )
            startActivity(intent)
        }

        geomagneticLevelingAttitudeButton = findViewById(R.id.geomagnetic_leveling_attitude_button)
        geomagneticLevelingAttitudeButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, AttitudeEstimatorActivity::class.java)
            intent.putExtra(
                AttitudeEstimatorActivity.ESTIMATOR_TYPE,
                AttitudeEstimatorActivity.GEOMAGNETIC
            )
            startActivity(intent)
        }

        gravityLevelingButton = findViewById(R.id.gravity_leveling_button)
        gravityLevelingButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, LevelingEstimatorActivity::class.java)
            intent.putExtra(LevelingEstimatorActivity.USE_ACCELEROMETER, false)
            startActivity(intent)
        }

        accelerometerLowPassFilterLevelingButton =
            findViewById(R.id.accelerometer_low_pass_filter_leveling_button)
        accelerometerLowPassFilterLevelingButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, LevelingEstimatorActivity::class.java)
            intent.putExtra(LevelingEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                LevelingEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER
            )
            intent.putExtra(
                LevelingEstimatorActivity.AVERAGING_FILTER_TYPE,
                LevelingEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            startActivity(intent)
        }

        accelerometerMeanFilterLevelingButton =
            findViewById(R.id.accelerometer_mean_filter_leveling_button)
        accelerometerMeanFilterLevelingButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, LevelingEstimatorActivity::class.java)
            intent.putExtra(LevelingEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                LevelingEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER
            )
            intent.putExtra(
                LevelingEstimatorActivity.AVERAGING_FILTER_TYPE,
                LevelingEstimatorActivity.MEAN_AVERAGING_FILTER
            )
            startActivity(intent)
        }

        accelerometerMedianFilterLevelingButton =
            findViewById(R.id.accelerometer_median_filter_leveling_button)
        accelerometerMedianFilterLevelingButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, LevelingEstimatorActivity::class.java)
            intent.putExtra(LevelingEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                LevelingEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER
            )
            intent.putExtra(
                LevelingEstimatorActivity.AVERAGING_FILTER_TYPE,
                LevelingEstimatorActivity.MEDIAN_AVERAGING_FILTER
            )
            startActivity(intent)
        }

        accelerometerUncalibratedLowPassFilterLevelingButton =
            findViewById(R.id.accelerometer_uncalibrated_low_pass_filter_leveling_button)
        accelerometerUncalibratedLowPassFilterLevelingButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, LevelingEstimatorActivity::class.java)
            intent.putExtra(LevelingEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                LevelingEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                LevelingEstimatorActivity.AVERAGING_FILTER_TYPE,
                LevelingEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            startActivity(intent)
        }

        accelerometerUncalibratedMeanFilterLevelingButton =
            findViewById(R.id.accelerometer_uncalibrated_mean_filter_leveling_button)
        accelerometerUncalibratedMeanFilterLevelingButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, LevelingEstimatorActivity::class.java)
            intent.putExtra(LevelingEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                LevelingEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                LevelingEstimatorActivity.AVERAGING_FILTER_TYPE,
                LevelingEstimatorActivity.MEAN_AVERAGING_FILTER
            )
            startActivity(intent)
        }

        accelerometerUncalibratedMedianFilterLevelingButton =
            findViewById(R.id.accelerometer_uncalibrated_median_filter_leveling_button)
        accelerometerUncalibratedMedianFilterLevelingButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, LevelingEstimatorActivity::class.java)
            intent.putExtra(LevelingEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                LevelingEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                LevelingEstimatorActivity.AVERAGING_FILTER_TYPE,
                LevelingEstimatorActivity.MEDIAN_AVERAGING_FILTER
            )
            startActivity(intent)
        }

        gravityAccurateLevelingButton = findViewById(R.id.gravity_accurate_leveling_button)
        gravityAccurateLevelingButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, AccurateLevelingEstimatorActivity::class.java)
            intent.putExtra(AccurateLevelingEstimatorActivity.USE_ACCELEROMETER, false)
            startActivity(intent)
        }

        accelerometerLowPassFilterAccurateLevelingButton =
            findViewById(R.id.accelerometer_low_pass_filter_accurate_leveling_button)
        accelerometerLowPassFilterAccurateLevelingButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, AccurateLevelingEstimatorActivity::class.java)
            intent.putExtra(AccurateLevelingEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                AccurateLevelingEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER
            )
            intent.putExtra(
                AccurateLevelingEstimatorActivity.AVERAGING_FILTER_TYPE,
                AccurateLevelingEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            startActivity(intent)
        }

        accelerometerMeanFilterAccurateLevelingButton =
            findViewById(R.id.accelerometer_mean_filter_accurate_leveling_button)
        accelerometerMeanFilterAccurateLevelingButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, AccurateLevelingEstimatorActivity::class.java)
            intent.putExtra(AccurateLevelingEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                AccurateLevelingEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER
            )
            intent.putExtra(
                AccurateLevelingEstimatorActivity.AVERAGING_FILTER_TYPE,
                AccurateLevelingEstimatorActivity.MEAN_AVERAGING_FILTER
            )
            startActivity(intent)
        }

        accelerometerMedianFilterAccurateLevelingButton =
            findViewById(R.id.accelerometer_median_filter_accurate_leveling_button)
        accelerometerMedianFilterAccurateLevelingButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, AccurateLevelingEstimatorActivity::class.java)
            intent.putExtra(AccurateLevelingEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                AccurateLevelingEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER
            )
            intent.putExtra(
                AccurateLevelingEstimatorActivity.AVERAGING_FILTER_TYPE,
                AccurateLevelingEstimatorActivity.MEDIAN_AVERAGING_FILTER
            )
            startActivity(intent)
        }

        accelerometerUncalibratedLowPassFilterAccurateLevelingButton =
            findViewById(R.id.accelerometer_uncalibrated_low_pass_filter_accurate_leveling_button)
        accelerometerUncalibratedLowPassFilterAccurateLevelingButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, AccurateLevelingEstimatorActivity::class.java)
            intent.putExtra(AccurateLevelingEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                AccurateLevelingEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                AccurateLevelingEstimatorActivity.AVERAGING_FILTER_TYPE,
                AccurateLevelingEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            startActivity(intent)
        }

        accelerometerUncalibratedMeanFilterAccurateLevelingButton =
            findViewById(R.id.accelerometer_uncalibrated_mean_filter_accurate_leveling_button)
        accelerometerUncalibratedMeanFilterAccurateLevelingButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, AccurateLevelingEstimatorActivity::class.java)
            intent.putExtra(AccurateLevelingEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                AccurateLevelingEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                AccurateLevelingEstimatorActivity.AVERAGING_FILTER_TYPE,
                AccurateLevelingEstimatorActivity.MEAN_AVERAGING_FILTER
            )
            startActivity(intent)
        }

        accelerometerUncalibratedMedianFilterAccurateLevelingButton =
            findViewById(R.id.accelerometer_uncalibrated_median_filter_accurate_leveling_button)
        accelerometerUncalibratedMedianFilterAccurateLevelingButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, AccurateLevelingEstimatorActivity::class.java)
            intent.putExtra(AccurateLevelingEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                AccurateLevelingEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                AccurateLevelingEstimatorActivity.AVERAGING_FILTER_TYPE,
                AccurateLevelingEstimatorActivity.MEDIAN_AVERAGING_FILTER
            )
            startActivity(intent)
        }

        relativeGyroscopeAttitudeButton = findViewById(R.id.relative_gyroscope_attitude_button)
        relativeGyroscopeAttitudeButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, RelativeGyroscopeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                RelativeGyroscopeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            startActivity(intent)
        }

        relativeUncalibratedGyroscopeAttitudeButton =
            findViewById(R.id.relative_uncalibrated_gyroscope_attitude_button)
        relativeUncalibratedGyroscopeAttitudeButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, RelativeGyroscopeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                RelativeGyroscopeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED
            )
            startActivity(intent)
        }

        accurateRelativeGyroscopeAttitudeButton =
            findViewById(R.id.accurate_relative_gyroscope_attitude_button)
        accurateRelativeGyroscopeAttitudeButton?.setOnClickListener {
            val intent =
                Intent(
                    this@MainActivity,
                    AccurateRelativeGyroscopeAttitudeEstimatorActivity::class.java
                )
            intent.putExtra(
                AccurateRelativeGyroscopeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            startActivity(intent)
        }

        accurateRelativeUncalibratedGyroscopeAttitudeButton =
            findViewById(R.id.accurate_relative_uncalibrated_gyroscope_attitude_button)
        accurateRelativeUncalibratedGyroscopeAttitudeButton?.setOnClickListener {
            val intent =
                Intent(
                    this@MainActivity,
                    AccurateRelativeGyroscopeAttitudeEstimatorActivity::class.java
                )
            intent.putExtra(
                AccurateRelativeGyroscopeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorGravityLevelingButton =
            findViewById(R.id.leveled_relative_attitude_estimator_gravity_leveling_button)
        leveledRelativeAttitudeEstimatorGravityLevelingButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCELEROMETER,
                false
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                false
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                false
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorAccelerometerLowPassFilterLevelingButton =
            findViewById(R.id.leveled_relative_attitude_estimator_accelerometer_low_pass_filter_leveling_button)
        leveledRelativeAttitudeEstimatorAccelerometerLowPassFilterLevelingButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCELEROMETER,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                false
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                false
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorAccelerometerMeanFilterLevelingButton =
            findViewById(R.id.leveled_relative_attitude_estimator_accelerometer_mean_filter_leveling_button)
        leveledRelativeAttitudeEstimatorAccelerometerMeanFilterLevelingButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCELEROMETER,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                false
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                false
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorAccelerometerMedianFilterLevelingButton =
            findViewById(R.id.leveled_relative_attitude_estimator_accelerometer_median_filter_leveling_button)
        leveledRelativeAttitudeEstimatorAccelerometerMedianFilterLevelingButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCELEROMETER,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEDIAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                false
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                false
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorAccelerometerUncalibratedLowPassFilterLevelingButton =
            findViewById(R.id.leveled_relative_attitude_estimator_accelerometer_uncalibrated_low_pass_filter_leveling_button)
        leveledRelativeAttitudeEstimatorAccelerometerUncalibratedLowPassFilterLevelingButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCELEROMETER,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                false
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                false
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorAccelerometerUncalibratedMeanFilterLevelingButton =
            findViewById(R.id.leveled_relative_attitude_estimator_accelerometer_uncalibrated_mean_filter_leveling_button)
        leveledRelativeAttitudeEstimatorAccelerometerUncalibratedMeanFilterLevelingButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCELEROMETER,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                false
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                false
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorAccelerometerUncalibratedMedianFilterLevelingButton =
            findViewById(R.id.leveled_relative_attitude_estimator_accelerometer_uncalibrated_median_filter_leveling_button)
        leveledRelativeAttitudeEstimatorAccelerometerUncalibratedMedianFilterLevelingButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCELEROMETER,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEDIAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                false
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                false
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorUncalibratedGyroscopeButton =
            findViewById(R.id.leveled_relative_attitude_estimator_uncalibrated_gyroscope_button)
        leveledRelativeAttitudeEstimatorUncalibratedGyroscopeButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                false
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                false
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorGravityAccurateLevelingButton =
            findViewById(R.id.leveled_relative_attitude_estimator_gravity_accurate_leveling_button)
        leveledRelativeAttitudeEstimatorGravityAccurateLevelingButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCELEROMETER,
                false
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                false
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorAccelerometerLowPassFilterAccurateLevelingButton =
            findViewById(R.id.leveled_relative_attitude_estimator_accelerometer_low_pass_filter_accurate_leveling_button)
        leveledRelativeAttitudeEstimatorAccelerometerLowPassFilterAccurateLevelingButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCELEROMETER,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                false
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorAccelerometerMeanFilterAccurateLevelingButton =
            findViewById(R.id.leveled_relative_attitude_estimator_accelerometer_mean_filter_accurate_leveling_button)
        leveledRelativeAttitudeEstimatorAccelerometerMeanFilterAccurateLevelingButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCELEROMETER,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                false
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorAccelerometerMedianFilterAccurateLevelingButton =
            findViewById(R.id.leveled_relative_attitude_estimator_accelerometer_median_filter_accurate_leveling_button)
        leveledRelativeAttitudeEstimatorAccelerometerMedianFilterAccurateLevelingButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCELEROMETER,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEDIAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                false
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorAccelerometerUncalibratedLowPassFilterAccurateLevelingButton =
            findViewById(R.id.leveled_relative_attitude_estimator_accelerometer_uncalibrated_low_pass_filter_accurate_leveling_button)
        leveledRelativeAttitudeEstimatorAccelerometerUncalibratedLowPassFilterAccurateLevelingButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCELEROMETER,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                false
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorAccelerometerUncalibratedMeanFilterAccurateLevelingButton =
            findViewById(R.id.leveled_relative_attitude_estimator_accelerometer_uncalibrated_mean_filter_accurate_leveling_button)
        leveledRelativeAttitudeEstimatorAccelerometerUncalibratedMeanFilterAccurateLevelingButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCELEROMETER,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                false
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorAccelerometerUncalibratedMedianFilterAccurateLevelingButton =
            findViewById(R.id.leveled_relative_attitude_estimator_accelerometer_uncalibrated_median_filter_accurate_leveling_button)
        leveledRelativeAttitudeEstimatorAccelerometerUncalibratedMedianFilterAccurateLevelingButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCELEROMETER,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEDIAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                false
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorUncalibratedGyroscopeAccurateLevelingButton =
            findViewById(R.id.leveled_relative_attitude_estimator_uncalibrated_gyroscope_accurate_leveling_button)
        leveledRelativeAttitudeEstimatorUncalibratedGyroscopeAccurateLevelingButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                false
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorGravityAccurateAttitudeButton =
            findViewById(R.id.leveled_relative_attitude_estimator_gravity_accurate_attitude_button)
        leveledRelativeAttitudeEstimatorGravityAccurateAttitudeButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCELEROMETER,
                false
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                false
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                true
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorAccelerometerLowPassFilterAccurateAttitudeButton =
            findViewById(R.id.leveled_relative_attitude_estimator_accelerometer_low_pass_filter_accurate_attitude_button)
        leveledRelativeAttitudeEstimatorAccelerometerLowPassFilterAccurateAttitudeButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCELEROMETER,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                false
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                true
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorAccelerometerMeanFilterAccurateAttitudeButton =
            findViewById(R.id.leveled_relative_attitude_estimator_accelerometer_mean_filter_accurate_attitude_button)
        leveledRelativeAttitudeEstimatorAccelerometerMeanFilterAccurateAttitudeButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCELEROMETER,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                false
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                true
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorAccelerometerMedianFilterAccurateAttitudeButton =
            findViewById(R.id.leveled_relative_attitude_estimator_accelerometer_median_filter_accurate_attitude_button)
        leveledRelativeAttitudeEstimatorAccelerometerMedianFilterAccurateAttitudeButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCELEROMETER,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEDIAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                false
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                true
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorAccelerometerUncalibratedLowPassFilterAccurateAttitudeButton =
            findViewById(R.id.leveled_relative_attitude_estimator_accelerometer_uncalibrated_low_pass_filter_accurate_attitude_button)
        leveledRelativeAttitudeEstimatorAccelerometerUncalibratedLowPassFilterAccurateAttitudeButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCELEROMETER,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                false
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                true
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorAccelerometerUncalibratedMeanFilterAccurateAttitudeButton =
            findViewById(R.id.leveled_relative_attitude_estimator_accelerometer_uncalibrated_mean_filter_accurate_attitude_button)
        leveledRelativeAttitudeEstimatorAccelerometerUncalibratedMeanFilterAccurateAttitudeButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCELEROMETER,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                false
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                true
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorAccelerometerUncalibratedMedianFilterAccurateAttitudeButton =
            findViewById(R.id.leveled_relative_attitude_estimator_accelerometer_uncalibrated_median_filter_accurate_attitude_button)
        leveledRelativeAttitudeEstimatorAccelerometerUncalibratedMedianFilterAccurateAttitudeButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCELEROMETER,
                true
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEDIAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                false
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                true
            )
            startActivity(intent)
        }

        leveledRelativeAttitudeEstimatorUncalibratedGyroscopeAccurateAttitudeButton =
            findViewById(R.id.leveled_relative_attitude_estimator_uncalibrated_gyroscope_accurate_attitude_button)
        leveledRelativeAttitudeEstimatorUncalibratedGyroscopeAccurateAttitudeButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, LeveledRelativeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_LEVELING_ESTIMATOR,
                false
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.USE_ACCURATE_RELATIVE_GYROSCOPE_ATTITUDE_ESTIMATOR,
                true
            )
            startActivity(intent)
        }
    }
}