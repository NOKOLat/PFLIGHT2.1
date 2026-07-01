#include "../state_headers.hpp"
#include "STM32_DPS368/util/dps_config.h"


StateError CalibrationState::init(StateContext& context) {

    constexpr uint16_t IMU_CALIBRATION_SAMPLE_COUNT = 100;

    if(context.imu->StartCalibration(IMU_CALIBRATION_SAMPLE_COUNT) != 0){

        return StateError::UPDATE_FAILED_CRITICAL;
    }

    printf("calib0\n");
    return StateError::NONE;
}


StateError CalibrationState::update(StateContext& context) {

    if(!context.imu->IsCalibrationComplete()){

        bool is_data_ready = false;
        if(context.imu->CheckDataReady(is_data_ready) != 0){

            return StateError::UPDATE_FAILED_CRITICAL;
        }

        if(!is_data_ready){

            return StateError::NONE;
        }

        int16_t accel_raw[3] = {};
        int16_t gyro_raw[3] = {};
        if(context.imu->GetRawData(accel_raw, gyro_raw) != 0){

            return StateError::UPDATE_FAILED_CRITICAL;
        }

        if(context.imu->AddCalibrationData(accel_raw, gyro_raw) != 0){

            return StateError::UPDATE_FAILED_CRITICAL;
        }

        return StateError::NONE;
    }

    constexpr float DEG_TO_RAD = 0.017453292519943295f;

    uint8_t imu_ret = context.imu->GetData(context.accel_data.data(), context.gyro_data.data());
    if(imu_ret != 0){

        return StateError::UPDATE_FAILED_CRITICAL;
    }

    bool has_temperature = false;
    bool has_pressure = false;
    int16_t baro_ret = context.baro->getSingleContResult(context.temperature_c, has_temperature, context.pressure_pa, has_pressure);
    if(baro_ret != DPS__SUCCEEDED){

        return StateError::UPDATE_FAILED_CRITICAL;
    }

    if(!has_pressure){

        return StateError::NONE;
    }

    float gyro_rad[3] = {
        context.gyro_data[0] * DEG_TO_RAD,
        context.gyro_data[1] * DEG_TO_RAD,
        context.gyro_data[2] * DEG_TO_RAD
    };

    context.navigation_ekf->CalibrateSample(context.accel_data.data(), gyro_rad, context.pressure_pa);

    return StateError::NONE;
}


StateResult CalibrationState::evaluateNextState(StateContext& context) {

    if(!context.imu->IsCalibrationComplete() || !context.navigation_ekf->IsCalibrated()){

        return {StateChange::NO_STATE_CHANGE, StateID::CALIBRATION};
    }

    return {StateChange::STATE_CHANGE, StateID::PRE_ARM};
}


StateID CalibrationState::getStateID() const {

    return StateID::CALIBRATION;
}
