#include "../state_headers.hpp"


StateError DisArmState::init(StateContext& context) {

    // モーターを停止する
    if (context.pwm_manager) {

        context.pwm_manager->stop();
    }

    return StateError::NONE;
}


StateError DisArmState::update(StateContext& context) {

    return StateError::NONE;
}


StateResult DisArmState::evaluateNextState(StateContext& context) {

    // 処理が完了したらPreArmStateへ遷移
    return {StateChange::STATE_CHANGE, StateID::PRE_ARM};
}


StateID DisArmState::getStateID() const {

    return StateID::DIS_ARM;
}
