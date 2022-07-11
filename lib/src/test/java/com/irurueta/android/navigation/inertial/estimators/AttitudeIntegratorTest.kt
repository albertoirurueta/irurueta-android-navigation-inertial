package com.irurueta.android.navigation.inertial.estimators

import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.*
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator
import com.irurueta.navigation.inertial.navigators.ECEFInertialNavigator
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.assertTrue
import org.junit.Test

class AttitudeIntegratorTest {

    @Test
    fun integrationStep_whenInitialAttitude_computesExpectedResult() {
        var numValid = 0
        for (t in 0 until TIMES) {
            val randomizer = UniformRandomizer()
            val latitude =
                Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES))
            val longitude =
                Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES))
            val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)
            val nedPosition = NEDPosition(latitude, longitude, height)

            val roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))

            val cbn1 = CoordinateTransformation(
                roll1,
                pitch1,
                yaw1,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            )

            val nedFrame1 = NEDFrame(nedPosition, cbn1)
            val ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1)

            // compute new orientation after 1 second
            val roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val cbn2 = CoordinateTransformation(
                roll2,
                pitch2,
                yaw2,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            )

            val nedFrame2 = NEDFrame(nedPosition, cbn2)
            val ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2)

            // assume that body kinematics are constant during the time interval of 1 second
            val kinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                TIME_INTERVAL,
                ecefFrame2,
                ecefFrame1
            )

            val previousFrame = ECEFFrame(ecefFrame1)
            val currentFrame = ECEFFrame()

            val result = Quaternion()
            val wx = kinematics.angularRateX
            val wy = kinematics.angularRateY
            val wz = kinematics.angularRateZ
            for (i in 0 until NUM_SAMPLES) {
                ECEFInertialNavigator.navigateECEF(
                    TIME_INTERVAL_BETWEEN_SAMPLES,
                    previousFrame,
                    kinematics,
                    currentFrame
                )

                if (i > 0) {
                    val previousAttitude =
                        previousFrame.coordinateTransformation.asRotation().toQuaternion()
                    AttitudeIntegrator.integrationStep(
                        previousAttitude,
                        wx,
                        wy,
                        wz,
                        wx,
                        wy,
                        wz,
                        TIME_INTERVAL_BETWEEN_SAMPLES,
                        result
                    )
                }

                previousFrame.copyFrom(currentFrame)
            }

            val expected = currentFrame.coordinateTransformation.asRotation().toQuaternion()
            expected.normalize()

            val expected2 = ecefFrame2.coordinateTransformation.asRotation().toQuaternion()
            expected2.normalize()

            assertTrue(expected.equals(expected2, ABSOLUTE_ERROR))

            result.normalize()

            if (!result.equals(expected, ABSOLUTE_ERROR)) {
                continue
            }
            assertTrue(result.equals(expected, ABSOLUTE_ERROR))

            numValid++
            break
        }

        assertTrue(numValid > 0)
    }

    @Test
    fun integrationStep_whenNoInitialAttitude_computesExpectedResult() {
        var numValid = 0
        for (t in 0 until TIMES) {
            val randomizer = UniformRandomizer()
            val latitude =
                Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES))
            val longitude =
                Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES))
            val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)
            val nedPosition = NEDPosition(latitude, longitude, height)

            val roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))

            val cbn1 = CoordinateTransformation(
                roll1,
                pitch1,
                yaw1,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            )

            val nedFrame1 = NEDFrame(nedPosition, cbn1)
            val ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1)

            // compute new orientation after 1 second
            val roll2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val pitch2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val yaw2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val cbn2 = CoordinateTransformation(
                roll2,
                pitch2,
                yaw2,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            )

            val nedFrame2 = NEDFrame(nedPosition, cbn2)
            val ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2)

            // assume that body kinematics are constant during the time interval of 1 second
            val kinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                TIME_INTERVAL,
                ecefFrame2,
                ecefFrame1
            )

            val previousFrame = ECEFFrame(ecefFrame1)
            // set the identity as initial attitude
            previousFrame.coordinateTransformation = CoordinateTransformation(
                FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
            )

            val currentFrame = ECEFFrame()

            val result = Quaternion()
            val wx = kinematics.angularRateX
            val wy = kinematics.angularRateY
            val wz = kinematics.angularRateZ
            for (i in 0 until NUM_SAMPLES) {
                ECEFInertialNavigator.navigateECEF(
                    TIME_INTERVAL_BETWEEN_SAMPLES,
                    previousFrame,
                    kinematics,
                    currentFrame
                )

                if (i > 0) {
                    val previousAttitude =
                        previousFrame.coordinateTransformation.asRotation().toQuaternion()
                    AttitudeIntegrator.integrationStep(
                        previousAttitude,
                        wx,
                        wy,
                        wz,
                        wx,
                        wy,
                        wz,
                        TIME_INTERVAL_BETWEEN_SAMPLES,
                        result
                    )
                }

                previousFrame.copyFrom(currentFrame)
            }

            val expected = currentFrame.coordinateTransformation.asRotation().toQuaternion()
            expected.normalize()

            result.normalize()

            if (!result.equals(expected, ABSOLUTE_ERROR)) {
                continue
            }
            assertTrue(result.equals(expected, ABSOLUTE_ERROR))

            numValid++
            break
        }

        assertTrue(numValid > 0)
    }

    private companion object {
        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 10000.0

        const val MIN_ANGLE_DEGREES = -45.0
        const val MAX_ANGLE_DEGREES = 45.0

        const val TIME_INTERVAL = 1.0
        const val NUM_SAMPLES = 50
        const val TIME_INTERVAL_BETWEEN_SAMPLES = TIME_INTERVAL / NUM_SAMPLES

        const val ABSOLUTE_ERROR = 1e-2

        const val TIMES = 100
    }
}