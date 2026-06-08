#include "../state_headers.hpp"


StateError AutoFlightState::onInit(StateContext& context) {

    return StateError::NONE;
}


StateError AutoFlightState::onUpdate(StateContext& context) {

    return StateError::NONE;
}


StateResult AutoFlightState::onEvaluateNextState(StateContext& context) {

    // Armスイッチが無効になったらDisArmStateへ遷移
    if(context.sbus_data.arm != SwitchPosition::HIGH){

        return {StateChange::STATE_CHANGE, StateID::DIS_ARM};
    }

    // Auto Missionスイッチが無効になったらFlightStateへ遷移
    if(context.sbus_data.auto_mission != SwitchPosition::HIGH){

        return {StateChange::STATE_CHANGE, StateID::FLIGHT};
    }

    return {StateChange::NO_STATE_CHANGE, StateID::AUTO_FLIGHT};
}


StateID AutoFlightState::getStateID() const {

    return StateID::AUTO_FLIGHT;
}
