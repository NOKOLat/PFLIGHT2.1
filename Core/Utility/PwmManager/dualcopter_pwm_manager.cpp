#include "dualcopter_pwm_manager.hpp"
#include "../Config/dualcopter_config.hpp"
#include <algorithm>
#include <cstdio>


bool DualcopterPwmManager::init() {

    // モーター0~1をemplace（dualcopter_config.hppのTIM・チャンネル設定を参照）
    motors_[0].emplace(DualcopterConfig::MOTOR0.htim, DualcopterConfig::MOTOR0.channel);
    motors_[1].emplace(DualcopterConfig::MOTOR1.htim, DualcopterConfig::MOTOR1.channel);

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {

        motors_[i]->setPulseRange(PwmConfig::MOTOR_MIN_PULSE, PwmConfig::MOTOR_MAX_PULSE);
    }

    // サーボ0~3をemplace
    servos_[0].emplace(DualcopterConfig::SERVO0.htim, DualcopterConfig::SERVO0.channel);
    servos_[1].emplace(DualcopterConfig::SERVO1.htim, DualcopterConfig::SERVO1.channel);
    servos_[2].emplace(DualcopterConfig::SERVO2.htim, DualcopterConfig::SERVO2.channel);
    servos_[3].emplace(DualcopterConfig::SERVO3.htim, DualcopterConfig::SERVO3.channel);

    for (uint8_t i = 0; i < SERVO_COUNT; i++) {

        servos_[i]->setPulseRange(PwmConfig::SERVO_MIN_PULSE, PwmConfig::SERVO_MAX_PULSE);
    }

    // 初期化チェック
    if (!checkMotors({0, 1})) {

        printf("[DualcopterPwmManager] モーターの初期化に失敗しました\n");
        return false;
    }

    if (!checkServos({0, 1, 2, 3})) {

        printf("[DualcopterPwmManager] サーボの初期化に失敗しました\n");
        return false;
    }

    printf("[DualcopterPwmManager] 初期化が完了しました\n");
    return true;
}


void DualcopterPwmManager::mix(float throttle, float pid_pitch, float pid_roll, float pid_yaw) {

    // モーターはスロットルをそのまま出力（2重反転のため姿勢制御不要）
    throttle = std::clamp(throttle, 0.0f, 100.0f);
    motor_output_[0] = throttle;
    motor_output_[1] = throttle;

    // サーボ ミキシング計算
    servo_output_[0] = -pid_roll  + pid_yaw;
    servo_output_[1] = -pid_pitch + pid_yaw;
    servo_output_[2] = pid_roll   + pid_yaw;
    servo_output_[3] = pid_pitch  + pid_yaw;

    // サーボ出力を-90~90度の範囲にクランプ
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {

        servo_output_[i] = std::clamp(servo_output_[i], -90.0f, 90.0f);
    }
}


void DualcopterPwmManager::output() {

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {

        motors_[i]->setSpeed(motor_output_[i]);
    }

    for (uint8_t i = 0; i < SERVO_COUNT; i++) {

        servos_[i]->setAngle(servo_output_[i]);
    }
}


void DualcopterPwmManager::stop() {

    for (uint8_t i = 0; i < motors_.size(); i++) {

        if (motors_[i].has_value()) {

            motors_[i]->stop();
        }
    }

    for (uint8_t i = 0; i < servos_.size(); i++) {

        if (servos_[i].has_value()) {

            servos_[i]->neutral();
        }
    }
}
