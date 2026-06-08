#ifndef QUADCOPTER_PWM_MANAGER_HPP
#define QUADCOPTER_PWM_MANAGER_HPP

#include "pwm_manager.hpp"

/* ===========================================================
 * クワッドコプター用PWM管理クラス
 *
 * モーター4台 (TIM3 CH1-4) を使用する。
 * ミキシング計算式:
 *   motor[0] = throttle + pitch + roll + yaw  (前右)
 *   motor[1] = throttle + pitch - roll - yaw  (前左)
 *   motor[2] = throttle - pitch + roll - yaw  (後右)
 *   motor[3] = throttle - pitch - roll + yaw  (後左)
 * =========================================================== */
class QuadcopterPwmManager : public PwmManager {

    public:

        // モーター4台を初期化し、初期化チェックを行う
        // [return] bool 初期化成功: true、失敗: false
        bool init() override;

        // クワッドコプター ミキシング計算
        // [in] float throttle   - スロットル値 (0~100%)
        // [in] float pid_pitch  - PitchのPID出力
        // [in] float pid_roll   - RollのPID出力
        // [in] float pid_yaw    - YawのPID出力
        void mix(float throttle, float pid_pitch, float pid_roll, float pid_yaw) override;

        // motor_output_をモーター0~3に出力する
        void output() override;

        // モーター0~3をすべて停止する
        void stop() override;

    private:

        // 使用するモーター台数
        static constexpr uint8_t MOTOR_COUNT = 4;
};

#endif // QUADCOPTER_PWM_MANAGER_HPP
