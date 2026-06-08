#ifndef PWM_MANAGER_HPP
#define PWM_MANAGER_HPP

#include <array>
#include <optional>
#include <initializer_list>
#include <cstdint>
#include "STM32_Motor-Servo_Driver/motor_controller.hpp"
#include "STM32_Motor-Servo_Driver/servo_controller.hpp"

/* ===========================================================
 * PWM出力を管理する基底クラス
 *
 * モーター(最大8台)・サーボ(最大4台)をoptionalで保持し、
 * 派生クラスが必要な分だけ初期化・ミキシング・出力を実装する。
 *
 * 使用方法:
 *   1. 派生クラスでinit()をoverrideし、必要なモーター/サーボをemplace
 *   2. mix()でミキシング計算を行い、motor_output_/servo_output_に格納
 *   3. output()でモーター/サーボに出力
 *   4. stop()で全モーター/サーボを停止
 * =========================================================== */
class PwmManager {

    public:

        virtual ~PwmManager() = default;

        // 初期化（派生クラスで必要なmotor/servoをemplaceし検証する）
        // [return] bool 初期化成功: true、失敗: false
        virtual bool init() = 0;

        // ミキシング計算（motor_output_/servo_output_に結果を格納）
        // [in] float throttle   - スロットル値 (0~100%)
        // [in] float pid_pitch  - PitchのPID出力
        // [in] float pid_roll   - RollのPID出力
        // [in] float pid_yaw    - YawのPID出力
        virtual void mix(float throttle, float pid_pitch, float pid_roll, float pid_yaw) = 0;

        // motor_output_/servo_output_をモーター・サーボに出力する
        virtual void output() = 0;

        // 全有効モーター・サーボを停止する
        virtual void stop() = 0;

        // モーターを直接制御する（テスト・デバッグ用）
        // [in] uint8_t idx      - モーターインデックス (0~7)
        // [in] float speed_pct  - 速度 (0~100%)
        void setMotorDirect(uint8_t idx, float speed_pct);

        // サーボを直接制御する（テスト・デバッグ用）
        // [in] uint8_t idx      - サーボインデックス (0~3)
        // [in] float angle_deg  - 角度 (-90~90度)
        void setServoDirect(uint8_t idx, float angle_deg);

        // 有効なモーター数を返す（has_value() == true の台数）
        uint8_t getMotorCount() const;

        // 有効なサーボ数を返す（has_value() == true の台数）
        uint8_t getServoCount() const;

    protected:

        // モーターインスタンス（最大8台、optionalで遅延初期化）
        std::array<std::optional<MotorController>, 8> motors_;

        // サーボインスタンス（最大4台、optionalで遅延初期化）
        std::array<std::optional<ServoController>, 4> servos_;

        // モーター出力値 [%]
        std::array<float, 8> motor_output_{};

        // サーボ出力値 [度]
        std::array<float, 4> servo_output_{};

        // 指定インデックスのモーターがすべて有効かチェックする
        // [in] std::initializer_list<uint8_t> indices - チェックするインデックスリスト
        // [return] bool 全て有効かつ初期化済み: true、問題あり: false
        bool checkMotors(std::initializer_list<uint8_t> indices);

        // 指定インデックスのサーボがすべて有効かチェックする
        // [in] std::initializer_list<uint8_t> indices - チェックするインデックスリスト
        // [return] bool 全て有効かつ初期化済み: true、問題あり: false
        bool checkServos(std::initializer_list<uint8_t> indices);
};

#endif // PWM_MANAGER_HPP
