#include "../state_headers.hpp"
#include <cmath>
#include "STM32_DPS368/util/dps_config.h"


StateError FlightStateBase::init(StateContext& context) {

    if(context.cascade_pid_manager){

        context.cascade_pid_manager->reset();
    }

    context.pid_output.fill(0.0f);
    context.throttle = 0.0f;

    return onInit(context);
}


StateError FlightStateBase::update(StateContext& context) {

    constexpr float DEG_TO_RAD = 0.017453292519943295f;

    uint8_t ret = context.imu->GetData(context.accel_data.data(), context.gyro_data.data());
    if(ret != 0){

        return StateError::UPDATE_FAILED_CRITICAL;
    }

    bool has_temperature = false;
    bool has_pressure = false;
    int16_t baro_ret = context.baro->getSingleContResult(context.temperature_c, has_temperature, context.pressure_pa, has_pressure);
    if(baro_ret != DPS__SUCCEEDED){

        return StateError::UPDATE_FAILED_CRITICAL;
    }

    float gyro_rad[3] = {
        context.gyro_data[0] * DEG_TO_RAD,
        context.gyro_data[1] * DEG_TO_RAD,
        context.gyro_data[2] * DEG_TO_RAD
    };

    float pressure_for_ekf = -1.0f;
    if(has_pressure && (context.pressure_pa > 0.0f) && std::isfinite(context.pressure_pa)){

        pressure_for_ekf = context.pressure_pa;
    }

    if(!context.navigation_ekf->Update(context.accel_data.data(), gyro_rad, pressure_for_ekf)){

        return StateError::UPDATE_FAILED_CRITICAL;
    }

    float angle_data[3] = {};
    context.navigation_ekf->GetAnglesDeg(angle_data);
    context.angle.roll = angle_data[0];
    context.angle.pitch = angle_data[1];
    context.angle.yaw = angle_data[2];
    context.navigation_ekf->GetAltitudeData(context.altitude_data.data());

    StateError err = onUpdate(context);
    if(err != StateError::NONE){

        return err;
    }

    pwm_tick_ = !pwm_tick_;
    if(pwm_tick_){

        context.pwm_manager->mix(context.throttle, context.pid_output[0], -context.pid_output[1], -context.pid_output[2]);
        context.pwm_manager->output();
    }

    //printf("[Accel] X: %.3f m/s^2, Y: %.3f m/s^2, Z: %.3f m/s^2\n", context.accel_data[0], context.accel_data[1], context.accel_data[2]);
    printf("[FlightStateBase] Altitude: %.3f m, Velocity: %.3f m/s, Accel: %.3f m/s^2\n", context.altitude_data[0], context.altitude_data[1], context.altitude_data[2]);
    //printf("[FlightStateBase] Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg\n", context.angle.roll, context.angle.pitch, context.angle.yaw);

    return StateError::NONE;
}


StateResult FlightStateBase::evaluateNextState(StateContext& context) {

    return onEvaluateNextState(context);
}
