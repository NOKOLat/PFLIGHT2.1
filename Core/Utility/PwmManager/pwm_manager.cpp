#include "pwm_manager.hpp"
#include <cstdio>


bool PwmManager::checkMotors(std::initializer_list<uint8_t> indices) {

    for (uint8_t idx : indices) {

        // インデックス範囲チェック
        if (idx >= motors_.size()) {

            printf("[PwmManager] モーター インデックス %d が範囲外です\n", idx);
            return false;
        }

        // optional有効チェック・初期化チェック
        if (!motors_[idx].has_value() || !motors_[idx]->isInitialized()) {

            printf("[PwmManager] モーター %d が初期化されていません\n", idx);
            return false;
        }
    }

    return true;
}


bool PwmManager::checkServos(std::initializer_list<uint8_t> indices) {

    for (uint8_t idx : indices) {

        // インデックス範囲チェック
        if (idx >= servos_.size()) {

            printf("[PwmManager] サーボ インデックス %d が範囲外です\n", idx);
            return false;
        }

        // optional有効チェック・初期化チェック
        if (!servos_[idx].has_value() || !servos_[idx]->isInitialized()) {

            printf("[PwmManager] サーボ %d が初期化されていません\n", idx);
            return false;
        }
    }

    return true;
}


void PwmManager::setMotorDirect(uint8_t idx, float speed_pct) {

    if (idx < motors_.size() && motors_[idx].has_value()) {

        motors_[idx]->setSpeed(speed_pct);
    }
}


void PwmManager::setServoDirect(uint8_t idx, float angle_deg) {

    if (idx < servos_.size() && servos_[idx].has_value()) {

        servos_[idx]->setAngle(angle_deg);
    }
}


uint8_t PwmManager::getMotorCount() const {

    uint8_t count = 0;

    for (const auto& motor : motors_) {

        if (motor.has_value()) {

            count++;
        }
    }

    return count;
}


uint8_t PwmManager::getServoCount() const {

    uint8_t count = 0;

    for (const auto& servo : servos_) {

        if (servo.has_value()) {

            count++;
        }
    }

    return count;
}
