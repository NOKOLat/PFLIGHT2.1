#include "../state_headers.hpp"
#include <cmath>
#include "STM32_DPS368/util/dps_config.h"


StateError FlightStateBase::init(StateContext& context) {

    if (context.cascade_pid_manager) {

        context.cascade_pid_manager->reset();
    }

    context.pid_output.fill(0.0f);
    context.throttle = 0.0f;

    return onInit(context);
}


StateError FlightStateBase::update(StateContext& context) {

    constexpr float RAD_TO_DEG = 57.29577951308232f;
    constexpr float DEG_TO_RAD = 0.017453292519943295f;

    // imuデータを取得（加速度: m/s², ジャイロ: dps）
    uint8_t ret = context.imu->GetData(context.accel_data.data(), context.gyro_data.data());
    if(ret != 0){

        return StateError::UPDATE_FAILED_CRITICAL;
    }

    // 気圧データを取得（圧力: Pa）
    bool has_temperature, has_pressure;
    int16_t baro_ret = context.baro->getSingleContResult(context.temperature_c, has_temperature, context.pressure_pa, has_pressure);
    if(baro_ret != DPS__SUCCEEDED){

        return StateError::UPDATE_FAILED_CRITICAL;
    }

    // ジャイロを dps → rad/s に変換
    float gyro_rad[3] = {context.gyro_data[0] * DEG_TO_RAD, context.gyro_data[1] * DEG_TO_RAD, context.gyro_data[2] * DEG_TO_RAD};

    // EKF 更新
    AttitudeEKF_Update(&context.ekf.value(), context.accel_data.data(), gyro_rad);

    // 推定角度を context に格納（単位: deg）
    context.angle.roll  = AttitudeEKF_GetRoll(&context.ekf.value())  * RAD_TO_DEG;
    context.angle.pitch = AttitudeEKF_GetPitch(&context.ekf.value()) * RAD_TO_DEG;
    context.angle.yaw   = AttitudeEKF_GetYaw(&context.ekf.value())   * RAD_TO_DEG;

    // 派生クラスの処理（throttle・pid_outputをcontextに書き込む）
    StateError err = onUpdate(context);
    if (err != StateError::NONE) {

        return err;
    }

    // ミキシング計算・PWM出力(2ループに1回)
    pwm_tick_ = !pwm_tick_;
    if (pwm_tick_) {

    	// センサーの向き依存の方向修正をマイナスでやる
        context.pwm_manager->mix(context.throttle, context.pid_output[0], -context.pid_output[1], -context.pid_output[2]);
        context.pwm_manager->output();
    }

    //debug angle
    //printf("[FlightStateBase] Angle: %3.3f, %3.3f, %3.3f deg\n", context.angle.roll, context.angle.pitch, context.angle.yaw);

    //debug pressure
    //context.publish_log("[FlightStateBase] Pressure: %f Pa, Temperature: %f °C", context.pressure_pa, context.temperature_c);

    return StateError::NONE;
}


StateResult FlightStateBase::evaluateNextState(StateContext& context) {

    return onEvaluateNextState(context);
}
