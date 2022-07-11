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
                Intent(this@MainActivity, AccurateRelativeGyroscopeAttitudeEstimatorActivity::class.java)
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
                Intent(this@MainActivity, AccurateRelativeGyroscopeAttitudeEstimatorActivity::class.java)
            intent.putExtra(
                AccurateRelativeGyroscopeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED
            )
            startActivity(intent)
        }
    }
}