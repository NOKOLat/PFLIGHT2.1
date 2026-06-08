#include "../state_headers.hpp"
#include <algorithm>


namespace {

constexpr float PREFLIGHT_SERVO_SCALE = 0.4f;
constexpr float SERVO_MIN_ANGLE_DEG = -90.0f;
constexpr float SERVO_MAX_ANGLE_DEG = 90.0f;

}


StateError PreFlightState::init(StateContext& context) {

    return StateError::NONE;
}


StateError PreFlightState::update(StateContext& context) {

    const float pitch = context.sbus_data.pitch * PREFLIGHT_SERVO_SCALE;
    const float roll = -context.sbus_data.roll * PREFLIGHT_SERVO_SCALE;
    const float yaw = -context.sbus_data.yaw * PREFLIGHT_SERVO_SCALE;

    context.pwm_manager->setServoDirect(0, std::clamp(-roll + yaw, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG));
    context.pwm_manager->setServoDirect(1, std::clamp(-pitch + yaw, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG));
    context.pwm_manager->setServoDirect(2, std::clamp( roll + yaw, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG));
    context.pwm_manager->setServoDirect(3, std::clamp( pitch + yaw, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG));

    return StateError::NONE;
}


StateResult PreFlightState::evaluateNextState(StateContext& context) {

    if (context.sbus_data.arm != SwitchPosition::HIGH) {

        return {StateChange::STATE_CHANGE, StateID::PRE_ARM};
    }

    if (context.sbus_data.safety == SwitchPosition::HIGH) {

        return {StateChange::STATE_CHANGE, StateID::FLIGHT};
    }

    return {StateChange::NO_STATE_CHANGE, StateID::PRE_FLIGHT};
}


StateID PreFlightState::getStateID() const {

    return StateID::PRE_FLIGHT;
}
