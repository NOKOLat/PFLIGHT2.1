#include "../state_headers.hpp"


StateError ArmState::init(StateContext& context) {

    return StateError::NONE;
}


StateError ArmState::update(StateContext& context) {

    if (!context.pwm_manager->init()) {

        printf("[ArmState] PwmManager init failed\n");
        return StateError::UPDATE_FAILED_CRITICAL;
    }

    const uint8_t servo_count = context.pwm_manager->getServoCount();
    const uint8_t motor_count = context.pwm_manager->getMotorCount();

    for (uint8_t i = 0; i < servo_count; i++) {

        context.pwm_manager->setServoDirect(i, 0.0f);
    }

    for (uint8_t i = 0; i < motor_count; i++) {

        context.pwm_manager->setMotorDirect(i, 0.0f);
    }

    return StateError::NONE;
}


StateResult ArmState::evaluateNextState(StateContext& context) {

    return {StateChange::STATE_CHANGE, StateID::PRE_FLIGHT};
}


StateID ArmState::getStateID() const {

    return StateID::ARM;
}
