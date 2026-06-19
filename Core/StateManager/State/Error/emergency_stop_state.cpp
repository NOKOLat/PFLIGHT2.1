#include "../state_headers.hpp"

void EmergencyStopState::stopMotor(StateContext& context) {

    if (context.pwm_manager) {

        context.pwm_manager->stop();
    }
}

StateError EmergencyStopState::init(StateContext& context) {

    stopMotor(context);
    while(1){

    	;
    }

    return StateError::NONE;
}

StateError EmergencyStopState::update(StateContext& context) {

    stopMotor(context);
    while(1){

    	;
    }

    return StateError::NONE;
}


StateResult EmergencyStopState::evaluateNextState(StateContext& context) {

    return {StateChange::NO_STATE_CHANGE, StateID::EMERGENCY_STOP};
}


StateID EmergencyStopState::getStateID() const {

    return StateID::EMERGENCY_STOP;
}
