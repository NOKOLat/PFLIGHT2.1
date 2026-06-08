#include "../state_headers.hpp"


StateError PreArmState::init(StateContext& context) {

    return StateError::NONE;
}


StateError PreArmState::update(StateContext& context) {

    return StateError::NONE;
}


StateResult PreArmState::evaluateNextState(StateContext& context) {

    // Armスイッチが有効になったらArmStateへ遷移
    if(context.sbus_data.arm == SwitchPosition::HIGH){

        return {StateChange::STATE_CHANGE, StateID::ARM};
    }

    return {StateChange::NO_STATE_CHANGE, StateID::PRE_ARM};
}


StateID PreArmState::getStateID() const {

    return StateID::PRE_ARM;
}
