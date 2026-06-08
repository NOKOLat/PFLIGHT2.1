#ifndef DUALCOPTER_CONFIG_HPP
#define DUALCOPTER_CONFIG_HPP

#include "pwm_config.hpp"
#include "tim.h"

/* ===========================================================
 * DualCopter タイマー・チャンネル設定
 *
 * Motor 0: htim1 / TIM_CHANNEL_1 (PA8)
 * Motor 1: htim1 / TIM_CHANNEL_2 (PA9)
 *
 * Servo  0: htim3 / TIM_CHANNEL_1 (PC6)
 * Servo  1: htim3 / TIM_CHANNEL_2 (PC7)
 * Servo  2: htim3 / TIM_CHANNEL_3 (PC8)
 * Servo  3: htim3 / TIM_CHANNEL_4 (PC9)
 * =========================================================== */
namespace DualcopterConfig {

    inline PwmConfig::PwmChannelConfig MOTOR0 = { &htim12, TIM_CHANNEL_1 };
    inline PwmConfig::PwmChannelConfig MOTOR1 = { &htim12, TIM_CHANNEL_2 };

    inline PwmConfig::PwmChannelConfig SERVO0 = { &htim1, TIM_CHANNEL_1 };
    inline PwmConfig::PwmChannelConfig SERVO1 = { &htim1, TIM_CHANNEL_2 };
    inline PwmConfig::PwmChannelConfig SERVO2 = { &htim1, TIM_CHANNEL_3 };
    inline PwmConfig::PwmChannelConfig SERVO3 = { &htim1, TIM_CHANNEL_4 };

} // namespace DualcopterConfig

#endif // DUALCOPTER_CONFIG_HPP
