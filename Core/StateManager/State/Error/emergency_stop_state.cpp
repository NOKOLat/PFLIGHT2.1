#include "../state_headers.hpp"

StateError EmergencyStopState::init(StateContext& context) {

    // 緊急停止: モーターを即時停止する
    if (context.pwm_manager) {

        context.pwm_manager->stop();
    }

    return StateError::NONE;
}

StateError EmergencyStopState::update(StateContext& context) {

    // 緊急停止: モーターを即時停止する
    if (context.pwm_manager) {

        context.pwm_manager->stop();
    }

    return StateError::NONE;
}


StateResult EmergencyStopState::evaluateNextState(StateContext& context) {

    return {StateChange::NO_STATE_CHANGE, StateID::EMERGENCY_STOP};
}


StateID EmergencyStopState::getStateID() const {
    
    return StateID::EMERGENCY_STOP;
}
