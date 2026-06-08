#ifndef PWM_CONFIG_HPP
#define PWM_CONFIG_HPP

#include "tim.h"
#include <cstdint>

namespace PwmConfig {

    // タイマーハンドルとチャンネルをまとめた設定構造体
    struct PwmChannelConfig {
        TIM_HandleTypeDef* htim;
        uint32_t           channel;
    };

    // ===== モーター パルス幅設定 =====
    static constexpr uint32_t MOTOR_MIN_PULSE = 1000;  // 最小パルス幅 [us] (0%)
    static constexpr uint32_t MOTOR_MAX_PULSE = 2000;  // 最大パルス幅 [us] (100%)

    // ===== サーボ パルス幅設定 =====
    static constexpr uint32_t SERVO_MIN_PULSE = 1000;  // 最小パルス幅 [us] (-90度)
    static constexpr uint32_t SERVO_MAX_PULSE = 2000;  // 最大パルス幅 [us] (+90度)

} // namespace PwmConfig

// タイマー・チャンネルの機体別設定は各機体のconfigファイルに記載する
// 例: quadcopter_config.hpp, dualcopter_config.hpp

#endif // PWM_CONFIG_HPP
