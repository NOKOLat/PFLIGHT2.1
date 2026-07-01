/*
 * navigation_ekf.h
 *
 *  Created on: Jun 19, 2026
 *      Author: Codex
 */

#ifndef NAVIGATION_EKF_H_
#define NAVIGATION_EKF_H_

#include <cstdint>

class NavigationEKF{

    public:

        void Init(float dt);
        void CalibrateSample(const float accel[3], const float gyro_rad[3], float pressure_pa);
        bool IsCalibrated() const { return calibrated_; }
        bool Update(const float accel[3], const float gyro_rad[3], float pressure_pa);
        void GetAnglesDeg(float out[3]) const;
        void GetAltitudeData(float out[3]) const;

    private:

        static constexpr int ERROR_STATE_SIZE = 9;
        static constexpr int OBS_SIZE_MAX = 4;
        static constexpr int CALIBRATION_SAMPLE_COUNT = 100;
        static constexpr int PRESSURE_AVERAGE_WINDOW_LOOPS = 10;

        // nominal state
        float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        float velocity_z_ = 0.0f;
        float altitude_ = 0.0f;
        float gyro_bias_[3] = {};
        float accel_z_bias_ = 0.0f;

        // filter state
        float covariance_[ERROR_STATE_SIZE * ERROR_STATE_SIZE] = {};
        float process_noise_[ERROR_STATE_SIZE * ERROR_STATE_SIZE] = {};

        // calibration state
        bool calibrated_ = false;
        uint32_t calibration_count_ = 0u;
        float accel_sum_[3] = {};
        float gyro_sum_[3] = {};
        float pressure_sum_ = 0.0f;
        float reference_pressure_ = 101325.0f;

        // pressure moving average
        float pressure_samples_[PRESSURE_AVERAGE_WINDOW_LOOPS] = {};
        float pressure_average_sum_ = 0.0f;
        uint32_t pressure_average_index_ = 0u;
        uint32_t pressure_average_count_ = 0u;

        // output
        float dt_ = 0.01f;
        float vertical_accel_ = 0.0f;

        void ResetFilter();
        void FinishCalibration();
        void PredictNominal(float q[4], float& velocity_z, float& altitude, const float gyro_bias[3], float accel_z_bias, const float accel[3], const float gyro_rad[3], float* vertical_accel_out) const;
        void PredictMeasurement(const float q[4], float altitude, bool use_baro, float out[OBS_SIZE_MAX]) const;
        void BuildJacobianF(const float accel[3], const float gyro_rad[3], float out[ERROR_STATE_SIZE * ERROR_STATE_SIZE]) const;
        void BuildJacobianH(bool use_baro, float out[OBS_SIZE_MAX * ERROR_STATE_SIZE], int obs_size) const;
        void ApplyErrorState(const float error[ERROR_STATE_SIZE]);
        void ApplyErrorStateTo(float q[4], float& velocity_z, float& altitude, float gyro_bias[3], float& accel_z_bias, const float error[ERROR_STATE_SIZE]) const;
        void ErrorBetween(const float q_ref[4], float velocity_ref, float altitude_ref, const float gyro_bias_ref[3], float accel_z_bias_ref,
                            const float q_test[4], float velocity_test, float altitude_test, const float gyro_bias_test[3], float accel_z_bias_test,
                            float out[ERROR_STATE_SIZE]) const;
        bool Correct(const float residual[OBS_SIZE_MAX], const float h_matrix[OBS_SIZE_MAX * ERROR_STATE_SIZE], const float r_diag[OBS_SIZE_MAX], int obs_size);
        float UpdatePressureAverage(float pressure_pa);
        float PressureToAltitude(float pressure_pa) const;
        void QuaternionToEuler(float& roll, float& pitch, float& yaw) const;
        void NormalizeQuaternion(float q[4]) const;
        void GravityDirectionBody(const float q[4], float out[3]) const;
        void SmallAngleToQuaternion(const float angle[3], float out[4]) const;
        void QuaternionMultiply(const float a[4], const float b[4], float out[4]) const;
};

#endif /* NAVIGATION_EKF_H_ */
