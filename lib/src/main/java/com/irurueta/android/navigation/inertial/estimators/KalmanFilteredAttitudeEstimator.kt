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
package com.irurueta.android.navigation.inertial.estimators

import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanConfig
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanFilteredEstimator
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanInitializerConfig
import com.irurueta.navigation.inertial.navigators.ECEFInertialNavigator

/**
 * @property initializerConfig contains uncertainty of initial position, attitude, velocity, acceleration, etc.
 * @property kalmanConfig contains configuration parameters for Kalman filter.
 */
class KalmanFilteredAttitudeEstimator(
    val initializerConfig: INSLooselyCoupledKalmanInitializerConfig = INSLooselyCoupledKalmanInitializerConfig(),
    val kalmanConfig: INSLooselyCoupledKalmanConfig = INSLooselyCoupledKalmanConfig()
) {

    //INSLooselyCoupledKalmanEpochEstimator
    //INSLooselyCoupledKalmanFilteredEstimator
    //INSLooselyCoupledKalmanFilteredEstimatorListener
    //INSLooselyCoupledKalmanInitializer
    //INSLooselyCoupledKalmanState

    /*
                    ECEFInertialNavigator.navigateECEF(propagationInterval, mFrame,
                        mCorrectedKinematics, mFrame);

     */

    private val kalmanEstimator =
        INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initializerConfig)

    private fun navigate() {
        //ECEFInertialNavigator.navigateECEF()
    }
}