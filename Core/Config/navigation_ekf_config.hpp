#ifndef NAVIGATION_EKF_CONFIG_HPP_
#define NAVIGATION_EKF_CONFIG_HPP_

#include "Navigation_EKF/navigation_ekf.h"

namespace NavigationEkfConfig {

inline constexpr NavigationEKFConfig CONFIG = {
    .calibration_sample_count = 100u,
    .covariance_initial = 1.0f,
    .q_angle = 1.0e-5f,
    .q_velocity = 1.0e-5f,
    .q_altitude = 1.0e-5f,
    .q_gyro_bias = 1.0e-7f,
    .q_accel_z_bias = 3.0e-6f,
    .r_accel_base = 3.0e-2f,
    .r_accel_gated = 5.0f,
    .r_baro = 2.5e-1f,
    .accel_gate = 2.0f,
    .calibration_accel_gate = 5.0e-1f,
    .calibration_gyro_gate = 1.0e-1f,
    .baro_altitude_step_gate = 10.0f,
    .pressure_min = 30000.0f,
    .pressure_max = 120000.0f
};

}

#endif /* NAVIGATION_EKF_CONFIG_HPP_ */
