package com.irurueta.android.navigation.inertial

import android.content.Intent
import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.widget.AppCompatButton

class MainActivity : AppCompatActivity() {
    private var absoluteAttitudeButton: AppCompatButton? = null

    private var relativeAttitudeButton: AppCompatButton? = null

    private var geomagneticAttitudeButton: AppCompatButton? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        absoluteAttitudeButton = findViewById(R.id.absolute_attitude_button)
        absoluteAttitudeButton?.setOnClickListener {
            val intent = Intent(this@MainActivity, AttitudeSensorCollectorActivity::class.java)
            intent.putExtra(
                AttitudeSensorCollectorActivity.SENSOR_TYPE,
                AttitudeSensorCollectorActivity.ABSOLUTE_ATTITUDE)
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
    }
}