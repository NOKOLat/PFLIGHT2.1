#ifndef DUALCOPTER_PWM_MANAGER_HPP
#define DUALCOPTER_PWM_MANAGER_HPP

#include "pwm_manager.hpp"

/* ===========================================================
 * デュアルコプター用PWM管理クラス
 *
 * モーター2台 (TIM1 CH1-2) : 2重反転、スロットルをそのまま出力
 * サーボ4台 (TIM12 CH1-2, TIM3 CH1-2) : 姿勢制御ミキシング
 *
 * ミキシング計算式:
 *   servo[0] = pitch + roll + yaw  (前右)
 *   servo[1] = pitch - roll - yaw  (前左)
 *   servo[2] = pitch + roll - yaw  (後右)
 *   servo[3] = pitch - roll + yaw  (後左)
 * =========================================================== */
class DualcopterPwmManager : public PwmManager {

    public:

        // モーター2台・サーボ4台を初期化し、初期化チェックを行う
        // [return] bool 初期化成功: true、失敗: false
        bool init() override;

        // デュアルコプター ミキシング計算
        // [in] float throttle   - スロットル値 (0~100%)
        // [in] float pid_pitch  - PitchのPID出力
        // [in] float pid_roll   - RollのPID出力
        // [in] float pid_yaw    - YawのPID出力
        void mix(float throttle, float pid_pitch, float pid_roll, float pid_yaw) override;

        // motor_output_をモーター0~1に、servo_output_をサーボ0~3に出力する
        void output() override;

        // モーター0~1を停止、サーボ0~3をニュートラルに移動する
        void stop() override;

    private:

        static constexpr uint8_t MOTOR_COUNT = 2;
        static constexpr uint8_t SERVO_COUNT = 4;
};

#endif // DUALCOPTER_PWM_MANAGER_HPP
