#include "../state_headers.hpp"
#include <cstdio>


StateError MotorServoTestState::init(StateContext& context) {

    counter_       = 0;
    current_index_ = 0;
    sub_step_      = 0;

    // サーボ・モーター数をキャッシュ
    servo_count_ = context.pwm_manager->getServoCount();
    motor_count_ = context.pwm_manager->getMotorCount();

    printf("[MotorServoTest] テスト開始 サーボ数:%d モーター数:%d\n",
           servo_count_, motor_count_);

    if (servo_count_ > 0) {

        // サーボ0番から開始: -45deg
        printf("[MotorServoTest] サーボ[%d] -45deg\n", current_index_);
        context.pwm_manager->setServoDirect(current_index_, -45.0f);
        sub_step_ = 0;

    } 
    else if (motor_count_ > 0) {

        // サーボなし: モーターテストへ直行
        printf("[MotorServoTest] サーボなし モーター[%d] 20%%\n", current_index_);
        context.pwm_manager->setMotorDirect(current_index_, 20.0f);
        sub_step_ = 3;

    } 
    else {

        // サーボもモーターもない
        printf("[MotorServoTest] テスト対象なし スキップ\n");
        sub_step_ = 4;
    }

    return StateError::NONE;
}


StateError MotorServoTestState::update(StateContext& context) {

    counter_++;

    switch (sub_step_) {

        // フェーズ0: サーボ -45deg 保持（1秒）
        case 0:
            if (counter_ >= ONE_SECOND_COUNT) {

                counter_  = 0;
                sub_step_ = 1;
                printf("[MotorServoTest] サーボ[%d] 0deg\n", current_index_);
                context.pwm_manager->setServoDirect(current_index_, 0.0f);
            }
            break;

        // フェーズ1: サーボ 0deg 保持（1秒）
        case 1:
            if (counter_ >= ONE_SECOND_COUNT) {

                counter_  = 0;
                sub_step_ = 2;
                printf("[MotorServoTest] サーボ[%d] 45deg\n", current_index_);
                context.pwm_manager->setServoDirect(current_index_, 45.0f);
            }
            break;

        // フェーズ2: サーボ 45deg 保持（1秒） → 中立に戻して次のサーボへ
        case 2:
            if (counter_ >= ONE_SECOND_COUNT) {

                counter_ = 0;

                // 中立に戻す
                context.pwm_manager->setServoDirect(current_index_, 0.0f);
                current_index_++;

                if (current_index_ < servo_count_) {

                    // 次のサーボへ
                    sub_step_ = 0;
                    printf("[MotorServoTest] サーボ[%d] -45deg\n", current_index_);
                    context.pwm_manager->setServoDirect(current_index_, -45.0f);

                } else if (motor_count_ > 0) {

                    // モーターテストへ
                    current_index_ = 0;
                    sub_step_ = 3;
                    printf("[MotorServoTest] モーター[%d] 20%%\n", current_index_);
                    context.pwm_manager->setMotorDirect(current_index_, 20.0f);

                } else {

                    // モーターなし: 完了
                    sub_step_ = 4;
                    printf("[MotorServoTest] テスト完了\n");
                }
            }
            break;

        // フェーズ3: モーター 20% 保持（1秒） → 停止して次のモーターへ
        case 3:
            if (counter_ >= ONE_SECOND_COUNT) {

                counter_ = 0;

                // 現在のモーターを停止
                context.pwm_manager->setMotorDirect(current_index_, 0.0f);
                current_index_++;

                if (current_index_ < motor_count_) {

                    // 次のモーターへ
                    printf("[MotorServoTest] モーター[%d] 20%%\n", current_index_);
                    context.pwm_manager->setMotorDirect(current_index_, 20.0f);

                } else {

                    // 全モーター完了
                    sub_step_ = 4;
                    printf("[MotorServoTest] テスト完了\n");
                }
            }
            break;

        // フェーズ4: 完了（evaluateNextState で遷移）
        default:
            break;
    }

    return StateError::NONE;
}


StateResult MotorServoTestState::evaluateNextState(StateContext& context) {

    if (sub_step_ >= 4) {

        return {StateChange::STATE_CHANGE, StateID::PRE_ARM};
    }

    return {StateChange::NO_STATE_CHANGE, StateID::MOTOR_SERVO_TEST};
}


StateID MotorServoTestState::getStateID() const {

    return StateID::MOTOR_SERVO_TEST;
}
