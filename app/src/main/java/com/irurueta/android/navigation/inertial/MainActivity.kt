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

import android.content.Intent
import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.widget.AppCompatButton
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.AttitudeSensorType
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorType

class MainActivity : AppCompatActivity() {

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

    private var leveledRelativeAttitudeEstimatorGravityAccurateLevelingButton: AppCompatButton? =
        null

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

    private var leveledRelativeAttitudeEstimatorUncalibratedGyroscopeAccurateLevelingButton: AppCompatButton? =
        null

    private var leveledRelativeAttitudeEstimatorGravityAccurateAttitudeButton: AppCompatButton? =
        null

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

    private var leveledRelativeAttitudeEstimatorUncalibratedGyroscopeAccurateAttitudeButton: AppCompatButton? =
        null

    private var geomagneticAttitudeEstimatorButton: AppCompatButton? = null

    private var geomagneticAttitudeEstimatorAccelerometerLowPassFilterButton: AppCompatButton? =
        null

    private var geomagneticAttitudeEstimatorAccelerometerMeanFilterButton: AppCompatButton? =
        null

    private var geomagneticAttitudeEstimatorAccelerometerMedianFilterButton: AppCompatButton? =
        null

    private var geomagneticAttitudeEstimatorAccelerometerUncalibratedLowPassButton: AppCompatButton? =
        null

    private var geomagneticAttitudeEstimatorAccelerometerUncalibratedMeanFilterButton: AppCompatButton? =
        null

    private var geomagneticAttitudeEstimatorAccelerometerUncalibratedMedianFilterButton: AppCompatButton? =
        null

    private var geomagneticAttitudeEstimatorUncalibratedMagnetometerButton: AppCompatButton? = null

    private var geomagneticAttitudeEstimatorWorldMagneticModelButton: AppCompatButton? = null

    private var fusedGeomagneticAttitudeEstimatorButton: AppCompatButton? = null

    private var fusedGeomagneticAttitudeEstimatorAccelerometerLowPassFilterButton: AppCompatButton? =
        null

    private var fusedGeomagneticAttitudeEstimatorAccelerometerMeanFilterButton: AppCompatButton? =
        null

    private var fusedGeomagneticAttitudeEstimatorAccelerometerMedianFilterButton: AppCompatButton? =
        null

    private var fusedGeomagneticAttitudeEstimatorAccelerometerUncalibratedLowPassButton: AppCompatButton? =
        null

    private var fusedGeomagneticAttitudeEstimatorAccelerometerUncalibratedMeanFilterButton: AppCompatButton? =
        null

    private var fusedGeomagneticAttitudeEstimatorAccelerometerUncalibratedMedianFilterButton: AppCompatButton? =
        null

    private var fusedGeomagneticAttitudeEstimatorUncalibratedMagnetometerButton: AppCompatButton? =
        null

    private var fusedGeomagneticAttitudeEstimatorUncalibratedGyroscopeButton: AppCompatButton? =
        null

    private var fusedGeomagneticAttitudeEstimatorWorldMagneticModelButton: AppCompatButton? = null

    private var doubleFusedGeomagneticAttitudeEstimatorButton: AppCompatButton? = null

    private var doubleFusedGeomagneticAttitudeEstimatorAccelerometerLowPassFilterButton: AppCompatButton? =
        null

    private var doubleFusedGeomagneticAttitudeEstimatorAccelerometerMeanFilterButton: AppCompatButton? =
        null

    private var doubleFusedGeomagneticAttitudeEstimatorAccelerometerMedianFilterButton: AppCompatButton? =
        null

    private var doubleFusedGeomagneticAttitudeEstimatorAccelerometerUncalibratedLowPassButton: AppCompatButton? =
        null

    private var doubleFusedGeomagneticAttitudeEstimatorAccelerometerUncalibratedMeanFilterButton: AppCompatButton? =
        null

    private var doubleFusedGeomagneticAttitudeEstimatorAccelerometerUncalibratedMedianFilterButton: AppCompatButton? =
        null

    private var doubleFusedGeomagneticAttitudeEstimatorUncalibratedMagnetometerButton: AppCompatButton? =
        null

    private var doubleFusedGeomagneticAttitudeEstimatorUncalibratedGyroscopeButton: AppCompatButton? =
        null

    private var doubleFusedGeomagneticAttitudeEstimatorWorldMagneticModelButton: AppCompatButton? = null

    private var androidAbsoluteAttitudeEstimatorButton: AppCompatButton? = null

    private var androidRelativeAttitudeEstimatorButton: AppCompatButton? = null

    private var poseEstimatorButton: AppCompatButton? = null

    private var localPoseEstimatorButton: AppCompatButton? = null

    private var relativePoseEstimatorButton: AppCompatButton? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

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
                AccelerometerSensorType.ACCELEROMETER
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
                AccelerometerSensorType.ACCELEROMETER
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
                AccelerometerSensorType.ACCELEROMETER
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
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
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
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
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
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
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
                AccelerometerSensorType.ACCELEROMETER
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
                AccelerometerSensorType.ACCELEROMETER
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
                AccelerometerSensorType.ACCELEROMETER
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
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
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
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
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
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
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
                GyroscopeSensorType.GYROSCOPE
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
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
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
                GyroscopeSensorType.GYROSCOPE
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
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
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
                GyroscopeSensorType.GYROSCOPE
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
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorType.GYROSCOPE
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
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorType.GYROSCOPE
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
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEDIAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorType.GYROSCOPE
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
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorType.GYROSCOPE
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
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorType.GYROSCOPE
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
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEDIAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorType.GYROSCOPE
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
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
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
                GyroscopeSensorType.GYROSCOPE
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
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorType.GYROSCOPE
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
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorType.GYROSCOPE
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
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEDIAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorType.GYROSCOPE
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
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorType.GYROSCOPE
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
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorType.GYROSCOPE
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
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEDIAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorType.GYROSCOPE
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
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
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
                GyroscopeSensorType.GYROSCOPE
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
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorType.GYROSCOPE
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
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorType.GYROSCOPE
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
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEDIAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorType.GYROSCOPE
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
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorType.GYROSCOPE
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
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorType.GYROSCOPE
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
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                LeveledRelativeAttitudeEstimatorActivity.MEDIAN_AVERAGING_FILTER
            )
            intent.putExtra(
                LeveledRelativeAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorType.GYROSCOPE
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
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
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

        geomagneticAttitudeEstimatorButton =
            findViewById(R.id.geomagnetic_attitude_estimator_gravity_button)
        geomagneticAttitudeEstimatorButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, GeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(GeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, false)
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            startActivity(intent)
        }

        geomagneticAttitudeEstimatorAccelerometerLowPassFilterButton =
            findViewById(R.id.geomagnetic_attitude_estimator_accelerometer_low_pass_filter_button)
        geomagneticAttitudeEstimatorAccelerometerLowPassFilterButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, GeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(GeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                GeomagneticAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            startActivity(intent)
        }

        geomagneticAttitudeEstimatorAccelerometerMeanFilterButton =
            findViewById(R.id.geomagnetic_attitude_estimator_accelerometer_mean_filter_button)
        geomagneticAttitudeEstimatorAccelerometerMeanFilterButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, GeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(GeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                GeomagneticAttitudeEstimatorActivity.MEAN_AVERAGING_FILTER
            )
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            startActivity(intent)
        }

        geomagneticAttitudeEstimatorAccelerometerMedianFilterButton =
            findViewById(R.id.geomagnetic_attitude_estimator_accelerometer_median_filter_button)
        geomagneticAttitudeEstimatorAccelerometerMedianFilterButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, GeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(GeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                GeomagneticAttitudeEstimatorActivity.MEDIAN_AVERAGING_FILTER
            )
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            startActivity(intent)
        }

        geomagneticAttitudeEstimatorAccelerometerUncalibratedLowPassButton =
            findViewById(R.id.geomagnetic_attitude_estimator_accelerometer_uncalibrated_low_pass_filter_button)
        geomagneticAttitudeEstimatorAccelerometerUncalibratedLowPassButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, GeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(GeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                GeomagneticAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            startActivity(intent)
        }

        geomagneticAttitudeEstimatorAccelerometerUncalibratedMeanFilterButton =
            findViewById(R.id.geomagnetic_attitude_estimator_accelerometer_uncalibrated_mean_filter_button)
        geomagneticAttitudeEstimatorAccelerometerUncalibratedMeanFilterButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, GeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(GeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                GeomagneticAttitudeEstimatorActivity.MEAN_AVERAGING_FILTER
            )
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            startActivity(intent)
        }

        geomagneticAttitudeEstimatorAccelerometerUncalibratedMedianFilterButton =
            findViewById(R.id.geomagnetic_attitude_estimator_accelerometer_uncalibrated_median_filter_button)
        geomagneticAttitudeEstimatorAccelerometerUncalibratedMedianFilterButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, GeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(GeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                GeomagneticAttitudeEstimatorActivity.MEDIAN_AVERAGING_FILTER
            )
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            startActivity(intent)
        }

        geomagneticAttitudeEstimatorUncalibratedMagnetometerButton =
            findViewById(R.id.geomagnetic_attitude_estimator_uncalibrated_magnetometer_accurate_attitude_button)
        geomagneticAttitudeEstimatorUncalibratedMagnetometerButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, GeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(GeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                GeomagneticAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
            )
            startActivity(intent)
        }

        geomagneticAttitudeEstimatorWorldMagneticModelButton =
            findViewById(R.id.geomagnetic_attitude_estimator_world_magnetic_model_button)
        geomagneticAttitudeEstimatorWorldMagneticModelButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, GeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(GeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                GeomagneticAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                GeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            intent.putExtra(GeomagneticAttitudeEstimatorActivity.USE_WORLD_MAGNETIC_MODEL, true)
            startActivity(intent)
        }

        fusedGeomagneticAttitudeEstimatorButton =
            findViewById(R.id.fused_geomagnetic_attitude_estimator_gravity_button)
        fusedGeomagneticAttitudeEstimatorButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, FusedGeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(FusedGeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, false)
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            startActivity(intent)
        }

        fusedGeomagneticAttitudeEstimatorAccelerometerLowPassFilterButton =
            findViewById(R.id.fused_geomagnetic_attitude_estimator_accelerometer_low_pass_filter_button)
        fusedGeomagneticAttitudeEstimatorAccelerometerLowPassFilterButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, FusedGeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(FusedGeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                FusedGeomagneticAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            startActivity(intent)
        }

        fusedGeomagneticAttitudeEstimatorAccelerometerMeanFilterButton =
            findViewById(R.id.fused_geomagnetic_attitude_estimator_accelerometer_mean_filter_button)
        fusedGeomagneticAttitudeEstimatorAccelerometerMeanFilterButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, FusedGeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(FusedGeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                FusedGeomagneticAttitudeEstimatorActivity.MEAN_AVERAGING_FILTER
            )
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            startActivity(intent)
        }

        fusedGeomagneticAttitudeEstimatorAccelerometerMedianFilterButton =
            findViewById(R.id.fused_geomagnetic_attitude_estimator_accelerometer_median_filter_button)
        fusedGeomagneticAttitudeEstimatorAccelerometerMedianFilterButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, FusedGeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(FusedGeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                FusedGeomagneticAttitudeEstimatorActivity.MEDIAN_AVERAGING_FILTER
            )
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            startActivity(intent)
        }

        fusedGeomagneticAttitudeEstimatorAccelerometerUncalibratedLowPassButton =
            findViewById(R.id.fused_geomagnetic_attitude_estimator_accelerometer_uncalibrated_low_pass_filter_button)
        fusedGeomagneticAttitudeEstimatorAccelerometerUncalibratedLowPassButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, FusedGeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(FusedGeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                FusedGeomagneticAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            startActivity(intent)
        }

        fusedGeomagneticAttitudeEstimatorAccelerometerUncalibratedMeanFilterButton =
            findViewById(R.id.fused_geomagnetic_attitude_estimator_accelerometer_uncalibrated_mean_filter_button)
        fusedGeomagneticAttitudeEstimatorAccelerometerUncalibratedMeanFilterButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, FusedGeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(FusedGeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                FusedGeomagneticAttitudeEstimatorActivity.MEAN_AVERAGING_FILTER
            )
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            startActivity(intent)
        }

        fusedGeomagneticAttitudeEstimatorAccelerometerUncalibratedMedianFilterButton =
            findViewById(R.id.fused_geomagnetic_attitude_estimator_accelerometer_uncalibrated_median_filter_button)
        fusedGeomagneticAttitudeEstimatorAccelerometerUncalibratedMedianFilterButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, FusedGeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(FusedGeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                FusedGeomagneticAttitudeEstimatorActivity.MEDIAN_AVERAGING_FILTER
            )
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            startActivity(intent)
        }

        fusedGeomagneticAttitudeEstimatorUncalibratedMagnetometerButton =
            findViewById(R.id.fused_geomagnetic_attitude_estimator_uncalibrated_magnetometer_accurate_attitude_button)
        fusedGeomagneticAttitudeEstimatorUncalibratedMagnetometerButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, FusedGeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(FusedGeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                FusedGeomagneticAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
            )
            startActivity(intent)
        }

        fusedGeomagneticAttitudeEstimatorUncalibratedGyroscopeButton =
            findViewById(R.id.fused_geomagnetic_attitude_estimator_uncalibrated_gyroscope_accurate_attitude_button)
        fusedGeomagneticAttitudeEstimatorUncalibratedGyroscopeButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, FusedGeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(FusedGeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                FusedGeomagneticAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
            )
            startActivity(intent)
        }

        fusedGeomagneticAttitudeEstimatorWorldMagneticModelButton =
            findViewById(R.id.fused_geomagnetic_attitude_estimator_world_magnetic_model_button)
        fusedGeomagneticAttitudeEstimatorWorldMagneticModelButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, FusedGeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(FusedGeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                FusedGeomagneticAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            intent.putExtra(
                FusedGeomagneticAttitudeEstimatorActivity.USE_WORLD_MAGNETIC_MODEL,
                true
            )
            startActivity(intent)
        }

        doubleFusedGeomagneticAttitudeEstimatorButton =
            findViewById(R.id.double_fused_geomagnetic_attitude_estimator_gravity_button)
        doubleFusedGeomagneticAttitudeEstimatorButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, DoubleFusedGeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(DoubleFusedGeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, false)
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            startActivity(intent)
        }

        doubleFusedGeomagneticAttitudeEstimatorAccelerometerLowPassFilterButton =
            findViewById(R.id.double_fused_geomagnetic_attitude_estimator_accelerometer_low_pass_filter_button)
        doubleFusedGeomagneticAttitudeEstimatorAccelerometerLowPassFilterButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, DoubleFusedGeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(DoubleFusedGeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                DoubleFusedGeomagneticAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            startActivity(intent)
        }

        doubleFusedGeomagneticAttitudeEstimatorAccelerometerMeanFilterButton =
            findViewById(R.id.double_fused_geomagnetic_attitude_estimator_accelerometer_mean_filter_button)
        doubleFusedGeomagneticAttitudeEstimatorAccelerometerMeanFilterButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, DoubleFusedGeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(DoubleFusedGeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                DoubleFusedGeomagneticAttitudeEstimatorActivity.MEAN_AVERAGING_FILTER
            )
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            startActivity(intent)
        }

        doubleFusedGeomagneticAttitudeEstimatorAccelerometerMedianFilterButton =
            findViewById(R.id.double_fused_geomagnetic_attitude_estimator_accelerometer_median_filter_button)
        doubleFusedGeomagneticAttitudeEstimatorAccelerometerMedianFilterButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, DoubleFusedGeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(DoubleFusedGeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                DoubleFusedGeomagneticAttitudeEstimatorActivity.MEDIAN_AVERAGING_FILTER
            )
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            startActivity(intent)
        }

        doubleFusedGeomagneticAttitudeEstimatorAccelerometerUncalibratedLowPassButton =
            findViewById(R.id.double_fused_geomagnetic_attitude_estimator_accelerometer_uncalibrated_low_pass_filter_button)
        doubleFusedGeomagneticAttitudeEstimatorAccelerometerUncalibratedLowPassButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, DoubleFusedGeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(DoubleFusedGeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                DoubleFusedGeomagneticAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            startActivity(intent)
        }

        doubleFusedGeomagneticAttitudeEstimatorAccelerometerUncalibratedMeanFilterButton =
            findViewById(R.id.double_fused_geomagnetic_attitude_estimator_accelerometer_uncalibrated_mean_filter_button)
        doubleFusedGeomagneticAttitudeEstimatorAccelerometerUncalibratedMeanFilterButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, DoubleFusedGeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(DoubleFusedGeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                DoubleFusedGeomagneticAttitudeEstimatorActivity.MEAN_AVERAGING_FILTER
            )
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            startActivity(intent)
        }

        doubleFusedGeomagneticAttitudeEstimatorAccelerometerUncalibratedMedianFilterButton =
            findViewById(R.id.double_fused_geomagnetic_attitude_estimator_accelerometer_uncalibrated_median_filter_button)
        doubleFusedGeomagneticAttitudeEstimatorAccelerometerUncalibratedMedianFilterButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, DoubleFusedGeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(DoubleFusedGeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
            )
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                DoubleFusedGeomagneticAttitudeEstimatorActivity.MEDIAN_AVERAGING_FILTER
            )
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            startActivity(intent)
        }

        doubleFusedGeomagneticAttitudeEstimatorUncalibratedMagnetometerButton =
            findViewById(R.id.double_fused_geomagnetic_attitude_estimator_uncalibrated_magnetometer_accurate_attitude_button)
        doubleFusedGeomagneticAttitudeEstimatorUncalibratedMagnetometerButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, DoubleFusedGeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(DoubleFusedGeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                DoubleFusedGeomagneticAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
            )
            startActivity(intent)
        }

        doubleFusedGeomagneticAttitudeEstimatorUncalibratedGyroscopeButton =
            findViewById(R.id.double_fused_geomagnetic_attitude_estimator_uncalibrated_gyroscope_accurate_attitude_button)
        doubleFusedGeomagneticAttitudeEstimatorUncalibratedGyroscopeButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, DoubleFusedGeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(DoubleFusedGeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                DoubleFusedGeomagneticAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.GYROSCOPE_SENSOR_TYPE,
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
            )
            startActivity(intent)
        }


        doubleFusedGeomagneticAttitudeEstimatorWorldMagneticModelButton =
            findViewById(R.id.double_fused_geomagnetic_attitude_estimator_world_magnetic_model_button)
        doubleFusedGeomagneticAttitudeEstimatorWorldMagneticModelButton?.setOnClickListener {
            val intent =
                Intent(this@MainActivity, DoubleFusedGeomagneticAttitudeEstimatorActivity::class.java)
            intent.putExtra(DoubleFusedGeomagneticAttitudeEstimatorActivity.USE_ACCELEROMETER, true)
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_SENSOR_TYPE,
                AccelerometerSensorType.ACCELEROMETER
            )
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.ACCELEROMETER_AVERAGING_FILTER_TYPE,
                DoubleFusedGeomagneticAttitudeEstimatorActivity.LOW_PASS_AVERAGING_FILTER
            )
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.MAGNETOMETER_SENSOR_TYPE,
                MagnetometerSensorType.MAGNETOMETER
            )
            intent.putExtra(
                DoubleFusedGeomagneticAttitudeEstimatorActivity.USE_WORLD_MAGNETIC_MODEL,
                true
            )
            startActivity(intent)
        }

        androidAbsoluteAttitudeEstimatorButton =
            findViewById(R.id.android_absolute_attitude_estimator_button)
        androidAbsoluteAttitudeEstimatorButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, AttitudeEstimatorActivity::class.java)
            intent.putExtra(
                AttitudeEstimatorActivity.ATTITUDE_SENSOR_TYPE,
                AttitudeSensorType.ABSOLUTE_ATTITUDE
            )
            startActivity(intent)
        }

        androidRelativeAttitudeEstimatorButton =
            findViewById(R.id.android_relative_attitude_estimator_button)
        androidRelativeAttitudeEstimatorButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, AttitudeEstimatorActivity::class.java)
            intent.putExtra(
                AttitudeEstimatorActivity.ATTITUDE_SENSOR_TYPE,
                AttitudeSensorType.RELATIVE_ATTITUDE
            )
            startActivity(intent)
        }

        poseEstimatorButton = findViewById(R.id.pose_estimator_button)
        poseEstimatorButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, PoseEstimatorActivity::class.java)
            startActivity(intent)
        }

        localPoseEstimatorButton = findViewById(R.id.local_pose_estimator_button)
        localPoseEstimatorButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, LocalPoseEstimatorActivity::class.java)
            startActivity(intent)
        }

        relativePoseEstimatorButton = findViewById(R.id.relative_pose_estimator_button)
        relativePoseEstimatorButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, RelativePoseEstimatorActivity::class.java)
            startActivity(intent)
        }
    }
}