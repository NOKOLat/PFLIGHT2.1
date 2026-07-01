/*
 * navigation_ekf.cpp
 *
 *  Created on: Jun 19, 2026
 *      Author: Codex
 */

#include "navigation_ekf.h"
#include <cmath>
#include <cstring>

namespace{

constexpr float G_CONST = 9.80665f;
constexpr float RAD_TO_DEG = 57.29577951308232f;
constexpr float FINITE_DIFF_EPS = 1.0e-4f;
constexpr float COV_INITIAL = 1.0f;
constexpr float Q_ANGLE = 1.0e-5f;
constexpr float Q_VELOCITY = 1.0e-5f;
constexpr float Q_ALTITUDE = 1.0e-5f;
constexpr float Q_GYRO_BIAS = 1.0e-7f;
constexpr float Q_ACCEL_Z_BIAS = 1.0e-5f;
constexpr float R_ACCEL_BASE = 3.0e-2f;
constexpr float R_ACCEL_GATED = 5.0f;
constexpr float R_BARO = 2.5e-1f;
constexpr float ACCEL_GATE = 2.0f;
constexpr float CALIBRATION_ACCEL_GATE = 5.0e-1f;
constexpr float CALIBRATION_GYRO_GATE = 1.0e-1f;
constexpr float BARO_ALTITUDE_STEP_GATE = 10.0f;
constexpr float PRESSURE_MIN = 30000.0f;
constexpr float PRESSURE_MAX = 120000.0f;
constexpr float COVARIANCE_MIN = 1.0e-9f;
constexpr float EPS = 1.0e-6f;

float Clamp(float value, float min_value, float max_value){

    if(value < min_value){

        return min_value;
    }
    if(value > max_value){

        return max_value;
    }

    return value;
}

bool IsFiniteVector3(const float value[3]){

    return value && std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

bool InvertMatrix(const float* input, float* inverse, int size){

    float work[4][8] = {};

    for(int i = 0; i < size; i++){

        for(int j = 0; j < size; j++){

            work[i][j] = input[i * size + j];
            work[i][j + size] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for(int col = 0; col < size; col++){

        int pivot = col;
        float max_abs = fabsf(work[col][col]);
        for(int row = col + 1; row < size; row++){

            float value_abs = fabsf(work[row][col]);
            if(value_abs > max_abs){

                max_abs = value_abs;
                pivot = row;
            }
        }

        if(max_abs < EPS){

            return false;
        }

        if(pivot != col){

            for(int j = 0; j < size * 2; j++){

                float tmp = work[col][j];
                work[col][j] = work[pivot][j];
                work[pivot][j] = tmp;
            }
        }

        float pivot_value = work[col][col];
        for(int j = 0; j < size * 2; j++){

            work[col][j] /= pivot_value;
        }

        for(int row = 0; row < size; row++){

            if(row == col){

                continue;
            }

            float factor = work[row][col];
            for(int j = 0; j < size * 2; j++){

                work[row][j] -= factor * work[col][j];
            }
        }
    }

    for(int i = 0; i < size; i++){

        for(int j = 0; j < size; j++){

            inverse[i * size + j] = work[i][j + size];
        }
    }

    return true;
}

}

/* @brief Initialize the navigation EKF.
 *
 * @param [in]dt Sampling period in seconds.
 */
void NavigationEKF::Init(float dt){

    dt_ = ((dt > 0.0f) && std::isfinite(dt)) ? dt : 0.01f;
    reference_pressure_ = 101325.0f;
    calibrated_ = false;
    calibration_count_ = 0u;
    pressure_sum_ = 0.0f;
    pressure_average_sum_ = 0.0f;
    pressure_average_index_ = 0u;
    pressure_average_count_ = 0u;
    vertical_accel_ = 0.0f;

    for(int i = 0; i < 3; i++){

        accel_sum_[i] = 0.0f;
        gyro_sum_[i] = 0.0f;
        gyro_bias_[i] = 0.0f;
    }

    for(int i = 0; i < PRESSURE_AVERAGE_WINDOW_LOOPS; i++){

        pressure_samples_[i] = 0.0f;
    }

    accel_z_bias_ = 0.0f;
    ResetFilter();
}

/* @brief Add one stationary calibration sample.
 *
 * @param [in]accel Acceleration in m/s^2.
 * @param [in]gyro_rad Angular velocity in rad/s.
 * @param [in]pressure_pa Pressure in Pa.
 */
void NavigationEKF::CalibrateSample(const float accel[3], const float gyro_rad[3], float pressure_pa){

    if(calibrated_ || !IsFiniteVector3(accel) || !IsFiniteVector3(gyro_rad)){

        return;
    }

    if(!std::isfinite(pressure_pa) || (pressure_pa < PRESSURE_MIN) || (pressure_pa > PRESSURE_MAX)){

        return;
    }

    float accel_norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    float gyro_norm = sqrtf(gyro_rad[0] * gyro_rad[0] + gyro_rad[1] * gyro_rad[1] + gyro_rad[2] * gyro_rad[2]);
    if((fabsf(accel_norm - G_CONST) > CALIBRATION_ACCEL_GATE) || (gyro_norm > CALIBRATION_GYRO_GATE)){

        return;
    }

    calibration_count_ ++;
    for(int i = 0; i < 3; i++){

        accel_sum_[i] += accel[i];
        gyro_sum_[i] += gyro_rad[i];
    }

    pressure_sum_ += pressure_pa;

    if(calibration_count_ >= CALIBRATION_SAMPLE_COUNT){

        FinishCalibration();
    }
}

/* @brief Run one prediction and correction step.
 *
 * @param [in]accel Acceleration in m/s^2.
 * @param [in]gyro_rad Angular velocity in rad/s.
 * @param [in]pressure_pa Pressure in Pa.
 *
 * @return bool true when the filter step completed.
 */
bool NavigationEKF::Update(const float accel[3], const float gyro_rad[3], float pressure_pa){

    if(!calibrated_ || !IsFiniteVector3(accel) || !IsFiniteVector3(gyro_rad)){

        return false;
    }

    float f_matrix[ERROR_STATE_SIZE * ERROR_STATE_SIZE] = {};
    BuildJacobianF(accel, gyro_rad, f_matrix);

    float next_covariance[ERROR_STATE_SIZE * ERROR_STATE_SIZE] = {};
    float tmp[ERROR_STATE_SIZE * ERROR_STATE_SIZE] = {};
    for(int row = 0; row < ERROR_STATE_SIZE; row++){

        for(int col = 0; col < ERROR_STATE_SIZE; col++){

            float sum = 0.0f;
            for(int k = 0; k < ERROR_STATE_SIZE; k++){

                sum += f_matrix[row * ERROR_STATE_SIZE + k] * covariance_[k * ERROR_STATE_SIZE + col];
            }
            tmp[row * ERROR_STATE_SIZE + col] = sum;
        }
    }
    for(int row = 0; row < ERROR_STATE_SIZE; row++){

        for(int col = 0; col < ERROR_STATE_SIZE; col++){

            float sum = 0.0f;
            for(int k = 0; k < ERROR_STATE_SIZE; k++){

                sum += tmp[row * ERROR_STATE_SIZE + k] * f_matrix[col * ERROR_STATE_SIZE + k];
            }
            if(!std::isfinite(sum)){

                return false;
            }
            next_covariance[row * ERROR_STATE_SIZE + col] = sum + process_noise_[row * ERROR_STATE_SIZE + col];
        }
    }

    for(int row = 0; row < ERROR_STATE_SIZE; row++){

        if(next_covariance[row * ERROR_STATE_SIZE + row] < COVARIANCE_MIN){

            next_covariance[row * ERROR_STATE_SIZE + row] = COVARIANCE_MIN;
        }

        for(int col = row + 1; col < ERROR_STATE_SIZE; col++){

            float symmetric_value = 0.5f * (next_covariance[row * ERROR_STATE_SIZE + col] + next_covariance[col * ERROR_STATE_SIZE + row]);
            next_covariance[row * ERROR_STATE_SIZE + col] = symmetric_value;
            next_covariance[col * ERROR_STATE_SIZE + row] = symmetric_value;
        }
    }
    memcpy(covariance_, next_covariance, sizeof(covariance_));

    PredictNominal(q_, velocity_z_, altitude_, gyro_bias_, accel_z_bias_, accel, gyro_rad, &vertical_accel_);
    NormalizeQuaternion(q_);

    if(!std::isfinite(q_[0]) || !std::isfinite(q_[1]) || !std::isfinite(q_[2]) || !std::isfinite(q_[3])
            || !std::isfinite(velocity_z_) || !std::isfinite(altitude_) || !std::isfinite(vertical_accel_)){

        return false;
    }

    float accel_norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    bool use_accel = std::isfinite(accel_norm) && (accel_norm > EPS);
    bool use_baro = std::isfinite(pressure_pa) && (pressure_pa >= PRESSURE_MIN) && (pressure_pa <= PRESSURE_MAX);
    float baro_altitude = 0.0f;

    if(use_baro){

        float raw_baro_altitude = PressureToAltitude(pressure_pa);
        float previous_average_pressure = (pressure_average_count_ > 0u)
                ? pressure_average_sum_ / (float)pressure_average_count_
                : reference_pressure_;
        float previous_baro_altitude = PressureToAltitude(previous_average_pressure);
        if(!std::isfinite(raw_baro_altitude) || !std::isfinite(previous_baro_altitude)
                || (fabsf(raw_baro_altitude - previous_baro_altitude) > BARO_ALTITUDE_STEP_GATE)){

            use_baro = false;
        }else{

            float averaged_pressure = UpdatePressureAverage(pressure_pa);
            baro_altitude = PressureToAltitude(averaged_pressure);
            if(!std::isfinite(baro_altitude)){

                use_baro = false;
            }
        }
    }

    if(!use_accel && !use_baro){

        return true;
    }

    int obs_size = use_baro ? 4 : 3;
    float measurement[OBS_SIZE_MAX] = {};
    float prediction[OBS_SIZE_MAX] = {};
    float residual[OBS_SIZE_MAX] = {};
    float h_matrix[OBS_SIZE_MAX * ERROR_STATE_SIZE] = {};
    float r_diag[OBS_SIZE_MAX] = {};

    if(use_accel){

        measurement[0] = accel[0] / accel_norm;
        measurement[1] = accel[1] / accel_norm;
        measurement[2] = accel[2] / accel_norm;

        float accel_error = fabsf(accel_norm - G_CONST);
        float accel_noise = (accel_error > ACCEL_GATE) ? R_ACCEL_GATED : R_ACCEL_BASE;
        r_diag[0] = accel_noise;
        r_diag[1] = accel_noise;
        r_diag[2] = accel_noise;
    }else{

        obs_size = use_baro ? 1 : 0;
    }

    if(use_baro){

        if(use_accel){

            measurement[3] = baro_altitude;
            r_diag[3] = R_BARO;
        }else{

            measurement[0] = baro_altitude;
            r_diag[0] = R_BARO;
        }
    }

    if(obs_size <= 0){

        return true;
    }

    PredictMeasurement(q_, altitude_, use_baro && use_accel, prediction);
    if(use_baro && !use_accel){

        prediction[0] = altitude_;
    }

    for(int i = 0; i < obs_size; i++){

        residual[i] = measurement[i] - prediction[i];
    }

    BuildJacobianH(use_baro && use_accel, h_matrix, obs_size);
    if(use_baro && !use_accel){

        for(int i = 0; i < ERROR_STATE_SIZE; i++){

            h_matrix[i] = 0.0f;
        }
        h_matrix[4] = 1.0f;
    }

    return Correct(residual, h_matrix, r_diag, obs_size);
}

/* @brief Get Euler angles in degrees.
 *
 * @param [out]out roll, pitch, yaw in degrees.
 */
void NavigationEKF::GetAnglesDeg(float out[3]) const{

    if(!out){

        return;
    }

    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    QuaternionToEuler(roll, pitch, yaw);

    out[0] = roll * RAD_TO_DEG;
    out[1] = pitch * RAD_TO_DEG;
    out[2] = yaw * RAD_TO_DEG;
}

/* @brief Get altitude related output.
 *
 * @param [out]out altitude, vertical velocity, vertical acceleration.
 */
void NavigationEKF::GetAltitudeData(float out[3]) const{

    if(!out){

        return;
    }

    out[0] = altitude_;
    out[1] = velocity_z_;
    out[2] = vertical_accel_;
}

void NavigationEKF::ResetFilter(){

    q_[0] = 1.0f;
    q_[1] = 0.0f;
    q_[2] = 0.0f;
    q_[3] = 0.0f;
    velocity_z_ = 0.0f;
    altitude_ = 0.0f;

    for(int i = 0; i < ERROR_STATE_SIZE * ERROR_STATE_SIZE; i++){

        covariance_[i] = 0.0f;
        process_noise_[i] = 0.0f;
    }

    for(int i = 0; i < ERROR_STATE_SIZE; i++){

        covariance_[i * ERROR_STATE_SIZE + i] = COV_INITIAL;
    }

    process_noise_[0 * ERROR_STATE_SIZE + 0] = Q_ANGLE;
    process_noise_[1 * ERROR_STATE_SIZE + 1] = Q_ANGLE;
    process_noise_[2 * ERROR_STATE_SIZE + 2] = Q_ANGLE;
    process_noise_[3 * ERROR_STATE_SIZE + 3] = Q_VELOCITY;
    process_noise_[4 * ERROR_STATE_SIZE + 4] = Q_ALTITUDE;
    process_noise_[5 * ERROR_STATE_SIZE + 5] = Q_GYRO_BIAS;
    process_noise_[6 * ERROR_STATE_SIZE + 6] = Q_GYRO_BIAS;
    process_noise_[7 * ERROR_STATE_SIZE + 7] = Q_GYRO_BIAS;
    process_noise_[8 * ERROR_STATE_SIZE + 8] = Q_ACCEL_Z_BIAS;
}

void NavigationEKF::FinishCalibration(){

    float sample_count = (float)calibration_count_;
    float accel_mean[3] = {};
    for(int i = 0; i < 3; i++){

        accel_mean[i] = accel_sum_[i] / sample_count;
        gyro_bias_[i] = gyro_sum_[i] / sample_count;
    }

    reference_pressure_ = pressure_sum_ / sample_count;
    if(!(reference_pressure_ > 0.0f) || !std::isfinite(reference_pressure_)){

        reference_pressure_ = 101325.0f;
    }

    pressure_average_sum_ = reference_pressure_ * (float)PRESSURE_AVERAGE_WINDOW_LOOPS;
    pressure_average_index_ = 0u;
    pressure_average_count_ = PRESSURE_AVERAGE_WINDOW_LOOPS;
    for(int i = 0; i < PRESSURE_AVERAGE_WINDOW_LOOPS; i++){

        pressure_samples_[i] = reference_pressure_;
    }

    float roll = atan2f(accel_mean[1], accel_mean[2]);
    float pitch = atan2f(-accel_mean[0], sqrtf(accel_mean[1] * accel_mean[1] + accel_mean[2] * accel_mean[2]));
    float yaw = 0.0f;

    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    q_[0] = cr * cp * cy + sr * sp * sy;
    q_[1] = sr * cp * cy - cr * sp * sy;
    q_[2] = cr * sp * cy + sr * cp * sy;
    q_[3] = cr * cp * sy - sr * sp * cy;
    NormalizeQuaternion(q_);

    float gravity_body[3] = {};
    GravityDirectionBody(q_, gravity_body);
    accel_z_bias_ = accel_mean[2] - gravity_body[2] * G_CONST;
    velocity_z_ = 0.0f;
    altitude_ = 0.0f;
    vertical_accel_ = 0.0f;
    calibrated_ = true;
}

void NavigationEKF::PredictNominal(float q[4], float& velocity_z, float& altitude, const float gyro_bias[3], float accel_z_bias, const float accel[3], const float gyro_rad[3], float* vertical_accel_out) const{

    float corrected_gyro[3] = {
        gyro_rad[0] - gyro_bias[0],
        gyro_rad[1] - gyro_bias[1],
        gyro_rad[2] - gyro_bias[2]
    };

    float omega_q[4] = {0.0f, corrected_gyro[0], corrected_gyro[1], corrected_gyro[2]};
    float q_dot[4] = {};
    QuaternionMultiply(q, omega_q, q_dot);
    for(int i = 0; i < 4; i++){

        q[i] += 0.5f * q_dot[i] * dt_;
    }
    NormalizeQuaternion(q);

    float accel_norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    if(accel_norm < EPS){

        if(vertical_accel_out){

            *vertical_accel_out = 0.0f;
        }
        altitude += velocity_z * dt_;
        return;
    }

    float accel_corrected[3] = {accel[0], accel[1], accel[2] - accel_z_bias};
    float gravity_body[3] = {};
    GravityDirectionBody(q, gravity_body);
    float world_z = gravity_body[0] * accel_corrected[0]
                  + gravity_body[1] * accel_corrected[1]
                  + gravity_body[2] * accel_corrected[2];

    float vertical_accel = world_z - G_CONST;
    if(vertical_accel_out){

        *vertical_accel_out = vertical_accel;
    }
    altitude += velocity_z * dt_ + 0.5f * vertical_accel * dt_ * dt_;
    velocity_z += vertical_accel * dt_;
}

void NavigationEKF::PredictMeasurement(const float q[4], float altitude, bool use_baro, float out[OBS_SIZE_MAX]) const{

    GravityDirectionBody(q, out);
    if(use_baro){

        out[3] = altitude;
    }
}

void NavigationEKF::BuildJacobianF(const float accel[3], const float gyro_rad[3], float out[ERROR_STATE_SIZE * ERROR_STATE_SIZE]) const{

    float q_base[4] = {q_[0], q_[1], q_[2], q_[3]};
    float velocity_base = velocity_z_;
    float altitude_base = altitude_;
    float gyro_bias_base[3] = {gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]};
    float accel_z_bias_base = accel_z_bias_;
    PredictNominal(q_base, velocity_base, altitude_base, gyro_bias_base, accel_z_bias_base, accel, gyro_rad, nullptr);

    for(int col = 0; col < ERROR_STATE_SIZE; col++){

        float error[ERROR_STATE_SIZE] = {};
        error[col] = FINITE_DIFF_EPS;

        float q_test[4] = {q_[0], q_[1], q_[2], q_[3]};
        float velocity_test = velocity_z_;
        float altitude_test = altitude_;
        float gyro_bias_test[3] = {gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]};
        float accel_z_bias_test = accel_z_bias_;
        ApplyErrorStateTo(q_test, velocity_test, altitude_test, gyro_bias_test, accel_z_bias_test, error);
        PredictNominal(q_test, velocity_test, altitude_test, gyro_bias_test, accel_z_bias_test, accel, gyro_rad, nullptr);

        float diff[ERROR_STATE_SIZE] = {};
        ErrorBetween(q_base, velocity_base, altitude_base, gyro_bias_base, accel_z_bias_base,
                     q_test, velocity_test, altitude_test, gyro_bias_test, accel_z_bias_test,
                     diff);

        for(int row = 0; row < ERROR_STATE_SIZE; row++){

            out[row * ERROR_STATE_SIZE + col] = diff[row] / FINITE_DIFF_EPS;
        }
    }
}

void NavigationEKF::BuildJacobianH(bool use_baro, float out[OBS_SIZE_MAX * ERROR_STATE_SIZE], int obs_size) const{

    float base[OBS_SIZE_MAX] = {};
    PredictMeasurement(q_, altitude_, use_baro, base);

    for(int col = 0; col < ERROR_STATE_SIZE; col++){

        float error[ERROR_STATE_SIZE] = {};
        error[col] = FINITE_DIFF_EPS;

        float q_test[4] = {q_[0], q_[1], q_[2], q_[3]};
        float velocity_test = velocity_z_;
        float altitude_test = altitude_;
        float gyro_bias_test[3] = {gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]};
        float accel_z_bias_test = accel_z_bias_;
        ApplyErrorStateTo(q_test, velocity_test, altitude_test, gyro_bias_test, accel_z_bias_test, error);

        float prediction[OBS_SIZE_MAX] = {};
        PredictMeasurement(q_test, altitude_test, use_baro, prediction);

        for(int row = 0; row < obs_size; row++){

            out[row * ERROR_STATE_SIZE + col] = (prediction[row] - base[row]) / FINITE_DIFF_EPS;
        }
    }
}

void NavigationEKF::ApplyErrorState(const float error[ERROR_STATE_SIZE]){

    ApplyErrorStateTo(q_, velocity_z_, altitude_, gyro_bias_, accel_z_bias_, error);
}

void NavigationEKF::ApplyErrorStateTo(float q[4], float& velocity_z, float& altitude, float gyro_bias[3], float& accel_z_bias, const float error[ERROR_STATE_SIZE]) const{

    float dq[4] = {};
    SmallAngleToQuaternion(error, dq);
    float q_new[4] = {};
    QuaternionMultiply(dq, q, q_new);
    for(int i = 0; i < 4; i++){

        q[i] = q_new[i];
    }
    NormalizeQuaternion(q);

    velocity_z += error[3];
    altitude += error[4];
    gyro_bias[0] += error[5];
    gyro_bias[1] += error[6];
    gyro_bias[2] += error[7];
    accel_z_bias += error[8];
}

void NavigationEKF::ErrorBetween(const float q_ref[4], float velocity_ref, float altitude_ref, const float gyro_bias_ref[3], float accel_z_bias_ref,
                                 const float q_test[4], float velocity_test, float altitude_test, const float gyro_bias_test[3], float accel_z_bias_test,
                                 float out[ERROR_STATE_SIZE]) const{

    float q_ref_inv[4] = {q_ref[0], -q_ref[1], -q_ref[2], -q_ref[3]};
    float dq[4] = {};
    QuaternionMultiply(q_test, q_ref_inv, dq);
    NormalizeQuaternion(dq);
    if(dq[0] < 0.0f){

        for(int i = 0; i < 4; i++){

            dq[i] = -dq[i];
        }
    }

    out[0] = 2.0f * dq[1];
    out[1] = 2.0f * dq[2];
    out[2] = 2.0f * dq[3];
    out[3] = velocity_test - velocity_ref;
    out[4] = altitude_test - altitude_ref;
    out[5] = gyro_bias_test[0] - gyro_bias_ref[0];
    out[6] = gyro_bias_test[1] - gyro_bias_ref[1];
    out[7] = gyro_bias_test[2] - gyro_bias_ref[2];
    out[8] = accel_z_bias_test - accel_z_bias_ref;
}

bool NavigationEKF::Correct(const float residual[OBS_SIZE_MAX], const float h_matrix[OBS_SIZE_MAX * ERROR_STATE_SIZE], const float r_diag[OBS_SIZE_MAX], int obs_size){

    float hp[OBS_SIZE_MAX * ERROR_STATE_SIZE] = {};
    for(int row = 0; row < obs_size; row++){

        for(int col = 0; col < ERROR_STATE_SIZE; col++){

            float sum = 0.0f;
            for(int k = 0; k < ERROR_STATE_SIZE; k++){

                sum += h_matrix[row * ERROR_STATE_SIZE + k] * covariance_[k * ERROR_STATE_SIZE + col];
            }
            hp[row * ERROR_STATE_SIZE + col] = sum;
        }
    }

    float s_matrix[OBS_SIZE_MAX * OBS_SIZE_MAX] = {};
    for(int row = 0; row < obs_size; row++){

        for(int col = 0; col < obs_size; col++){

            float sum = 0.0f;
            for(int k = 0; k < ERROR_STATE_SIZE; k++){

                sum += hp[row * ERROR_STATE_SIZE + k] * h_matrix[col * ERROR_STATE_SIZE + k];
            }
            s_matrix[row * obs_size + col] = sum + ((row == col) ? r_diag[row] : 0.0f);
        }
    }

    float s_inverse[OBS_SIZE_MAX * OBS_SIZE_MAX] = {};
    if(!InvertMatrix(s_matrix, s_inverse, obs_size)){

        return false;
    }

    float pht[ERROR_STATE_SIZE * OBS_SIZE_MAX] = {};
    for(int row = 0; row < ERROR_STATE_SIZE; row++){

        for(int col = 0; col < obs_size; col++){

            float sum = 0.0f;
            for(int k = 0; k < ERROR_STATE_SIZE; k++){

                sum += covariance_[row * ERROR_STATE_SIZE + k] * h_matrix[col * ERROR_STATE_SIZE + k];
            }
            pht[row * obs_size + col] = sum;
        }
    }

    float gain[ERROR_STATE_SIZE * OBS_SIZE_MAX] = {};
    for(int row = 0; row < ERROR_STATE_SIZE; row++){

        for(int col = 0; col < obs_size; col++){

            float sum = 0.0f;
            for(int k = 0; k < obs_size; k++){

                sum += pht[row * obs_size + k] * s_inverse[k * obs_size + col];
            }
            gain[row * obs_size + col] = sum;
        }
    }

    float error[ERROR_STATE_SIZE] = {};
    for(int row = 0; row < ERROR_STATE_SIZE; row++){

        float sum = 0.0f;
        for(int k = 0; k < obs_size; k++){

            sum += gain[row * obs_size + k] * residual[k];
        }
        error[row] = sum;
        if(!std::isfinite(error[row])){

            return false;
        }
    }

    float identity_minus_kh[ERROR_STATE_SIZE * ERROR_STATE_SIZE] = {};
    for(int row = 0; row < ERROR_STATE_SIZE; row++){

        for(int col = 0; col < ERROR_STATE_SIZE; col++){

            float sum = 0.0f;
            for(int k = 0; k < obs_size; k++){

                sum += gain[row * obs_size + k] * h_matrix[k * ERROR_STATE_SIZE + col];
            }
            identity_minus_kh[row * ERROR_STATE_SIZE + col] = ((row == col) ? 1.0f : 0.0f) - sum;
        }
    }

    float covariance_left[ERROR_STATE_SIZE * ERROR_STATE_SIZE] = {};
    for(int row = 0; row < ERROR_STATE_SIZE; row++){

        for(int col = 0; col < ERROR_STATE_SIZE; col++){

            float sum = 0.0f;
            for(int k = 0; k < ERROR_STATE_SIZE; k++){

                sum += identity_minus_kh[row * ERROR_STATE_SIZE + k] * covariance_[k * ERROR_STATE_SIZE + col];
            }
            covariance_left[row * ERROR_STATE_SIZE + col] = sum;
        }
    }

    float new_covariance[ERROR_STATE_SIZE * ERROR_STATE_SIZE] = {};
    for(int row = 0; row < ERROR_STATE_SIZE; row++){

        for(int col = 0; col < ERROR_STATE_SIZE; col++){

            float sum = 0.0f;
            for(int k = 0; k < ERROR_STATE_SIZE; k++){

                sum += covariance_left[row * ERROR_STATE_SIZE + k] * identity_minus_kh[col * ERROR_STATE_SIZE + k];
            }
            for(int k = 0; k < obs_size; k++){

                sum += gain[row * obs_size + k] * r_diag[k] * gain[col * obs_size + k];
            }

            if(!std::isfinite(sum)){

                return false;
            }
            new_covariance[row * ERROR_STATE_SIZE + col] = sum;
        }
    }

    for(int row = 0; row < ERROR_STATE_SIZE; row++){

        if(new_covariance[row * ERROR_STATE_SIZE + row] < COVARIANCE_MIN){

            new_covariance[row * ERROR_STATE_SIZE + row] = COVARIANCE_MIN;
        }

        for(int col = row + 1; col < ERROR_STATE_SIZE; col++){

            float symmetric_value = 0.5f * (new_covariance[row * ERROR_STATE_SIZE + col] + new_covariance[col * ERROR_STATE_SIZE + row]);
            new_covariance[row * ERROR_STATE_SIZE + col] = symmetric_value;
            new_covariance[col * ERROR_STATE_SIZE + row] = symmetric_value;
        }
    }

    ApplyErrorState(error);
    memcpy(covariance_, new_covariance, sizeof(covariance_));
    NormalizeQuaternion(q_);

    return true;
}

/* @brief Add pressure to the moving average.
 *
 * @param [in]pressure_pa Pressure in Pa.
 *
 * @return float Averaged pressure in Pa.
 */
float NavigationEKF::UpdatePressureAverage(float pressure_pa){

    if(pressure_average_count_ < PRESSURE_AVERAGE_WINDOW_LOOPS){

        pressure_samples_[pressure_average_index_] = pressure_pa;
        pressure_average_sum_ += pressure_pa;
        pressure_average_count_ ++;
    }else{

        pressure_average_sum_ -= pressure_samples_[pressure_average_index_];
        pressure_samples_[pressure_average_index_] = pressure_pa;
        pressure_average_sum_ += pressure_pa;
    }

    pressure_average_index_ = (pressure_average_index_ + 1u) % PRESSURE_AVERAGE_WINDOW_LOOPS;

    return pressure_average_sum_ / (float)pressure_average_count_;
}

/* @brief Convert pressure to relative altitude.
 *
 * @param [in]pressure_pa Pressure in Pa.
 *
 * @return float Relative altitude in meters.
 */
float NavigationEKF::PressureToAltitude(float pressure_pa) const{

    if(!(pressure_pa > 0.0f) || !(reference_pressure_ > 0.0f)){

        return altitude_;
    }

    return 44330.0f * (1.0f - powf(pressure_pa / reference_pressure_, 0.1903f));
}

void NavigationEKF::QuaternionToEuler(float& roll, float& pitch, float& yaw) const{

    float qw = q_[0];
    float qx = q_[1];
    float qy = q_[2];
    float qz = q_[3];

    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    roll = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (qw * qy - qz * qx);
    pitch = asinf(Clamp(sinp, -1.0f, 1.0f));

    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    yaw = atan2f(siny_cosp, cosy_cosp);
}

void NavigationEKF::NormalizeQuaternion(float q[4]) const{

    float norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if(norm < EPS){

        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return;
    }

    for(int i = 0; i < 4; i++){

        q[i] /= norm;
    }
}

void NavigationEKF::GravityDirectionBody(const float q[4], float out[3]) const{

    float qw = q[0];
    float qx = q[1];
    float qy = q[2];
    float qz = q[3];

    out[0] = 2.0f * (qx * qz - qw * qy);
    out[1] = 2.0f * (qy * qz + qw * qx);
    out[2] = qw * qw - qx * qx - qy * qy + qz * qz;
}

void NavigationEKF::SmallAngleToQuaternion(const float angle[3], float out[4]) const{

    float angle_norm = sqrtf(angle[0] * angle[0] + angle[1] * angle[1] + angle[2] * angle[2]);
    if(angle_norm < EPS){

        out[0] = 1.0f;
        out[1] = 0.5f * angle[0];
        out[2] = 0.5f * angle[1];
        out[3] = 0.5f * angle[2];
    }else{

        float half_angle = 0.5f * angle_norm;
        float scale = sinf(half_angle) / angle_norm;
        out[0] = cosf(half_angle);
        out[1] = angle[0] * scale;
        out[2] = angle[1] * scale;
        out[3] = angle[2] * scale;
    }
    NormalizeQuaternion(out);
}

void NavigationEKF::QuaternionMultiply(const float a[4], const float b[4], float out[4]) const{

    float result[4] = {};
    result[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    result[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    result[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    result[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];

    for(int i = 0; i < 4; i++){

        out[i] = result[i];
    }
}
