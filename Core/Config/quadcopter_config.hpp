#ifndef QUADCOPTER_CONFIG_HPP
#define QUADCOPTER_CONFIG_HPP

#include "pwm_config.hpp"
#include "tim.h"

/* ===========================================================
 * Quadcopter タイマー・チャンネル設定
 *
 * Motor 0: htim3 / TIM_CHANNEL_1 (PC6)
 * Motor 1: htim3 / TIM_CHANNEL_2 (PC7)
 * Motor 2: htim3 / TIM_CHANNEL_3 (PC8)
 * Motor 3: htim3 / TIM_CHANNEL_4 (PC9)
 * =========================================================== */
namespace QuadcopterConfig {

    inline PwmConfig::PwmChannelConfig MOTOR0 = { &htim3, TIM_CHANNEL_1 };
    inline PwmConfig::PwmChannelConfig MOTOR1 = { &htim3, TIM_CHANNEL_2 };
    inline PwmConfig::PwmChannelConfig MOTOR2 = { &htim3, TIM_CHANNEL_3 };
    inline PwmConfig::PwmChannelConfig MOTOR3 = { &htim3, TIM_CHANNEL_4 };

} // namespace QuadcopterConfig

#endif // QUADCOPTER_CONFIG_HPP
