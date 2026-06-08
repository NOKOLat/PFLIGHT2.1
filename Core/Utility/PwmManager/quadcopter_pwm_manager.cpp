#include "quadcopter_pwm_manager.hpp"
#include "../Config/quadcopter_config.hpp"
#include <algorithm>
#include <cstdio>


bool QuadcopterPwmManager::init() {

    // モーター0~3をemplace（quadcopter_config.hppのTIM・チャンネル設定を参照）
    motors_[0].emplace(QuadcopterConfig::MOTOR0.htim, QuadcopterConfig::MOTOR0.channel);
    motors_[1].emplace(QuadcopterConfig::MOTOR1.htim, QuadcopterConfig::MOTOR1.channel);
    motors_[2].emplace(QuadcopterConfig::MOTOR2.htim, QuadcopterConfig::MOTOR2.channel);
    motors_[3].emplace(QuadcopterConfig::MOTOR3.htim, QuadcopterConfig::MOTOR3.channel);

    // パルス幅範囲を設定
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {

        motors_[i]->setPulseRange(PwmConfig::MOTOR_MIN_PULSE, PwmConfig::MOTOR_MAX_PULSE);
    }

    // 初期化チェック
    if (!checkMotors({0, 1, 2, 3})) {

        printf("[QuadcopterPwmManager] モーターの初期化に失敗しました\n");
        return false;
    }

    printf("[QuadcopterPwmManager] モーターの初期化が完了しました\n");
    return true;
}


void QuadcopterPwmManager::mix(float throttle, float pid_pitch, float pid_roll, float pid_yaw) {

    // throttleは(0 ~ 75%)の範囲にクランプ（制御の余地を残すため）
    throttle = std::clamp(throttle, 0.0f, 75.0f);

    // クワッドコプター ミキシング計算
    motor_output_[0] = throttle + pid_pitch + pid_roll + pid_yaw;
    motor_output_[1] = throttle + pid_pitch - pid_roll - pid_yaw;
    motor_output_[2] = throttle - pid_pitch + pid_roll - pid_yaw;
    motor_output_[3] = throttle - pid_pitch - pid_roll + pid_yaw;

    // 出力値を0~100%の範囲にクランプ
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {

        motor_output_[i] = std::clamp(motor_output_[i], 0.0f, 100.0f);
    }
}


void QuadcopterPwmManager::output() {

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {

        motors_[i]->setSpeed(motor_output_[i]);
    }
}


void QuadcopterPwmManager::stop() {

    for (uint8_t i = 0; i < motors_.size(); i++) {

        if (motors_[i].has_value()) {

            motors_[i]->stop();
        }
    }
}
