#include "../state_headers.hpp"
#include "STM32_DPS368/util/dps_config.h"


StateError CalibrationState::init(StateContext& context) {

	printf("calib0\n");
    return StateError::NONE;
}


StateError CalibrationState::update(StateContext& context) {

	printf("calib\n");

    // icm42688pのキャリブレーション
    context.imu->Calibration(1000);

    while(!context.altitude_estimator->isCalibrated()){

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

        context.altitude_estimator->Calibration(context.pressure_pa, context.accel_data[2]);
    }

    return StateError::NONE;
}


StateResult CalibrationState::evaluateNextState(StateContext& context) {



    return {StateChange::STATE_CHANGE, StateID::PRE_ARM};
}


StateID CalibrationState::getStateID() const {

    return StateID::CALIBRATION;
}
