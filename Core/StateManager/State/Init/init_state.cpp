#include "../state_headers.hpp"
#include "../../../Config/system_config.hpp"
#include "../../../Config/navigation_ekf_config.hpp"
#include "../Config/sensor_config.hpp"
#include "../../../Utility/PwmManager/dualcopter_pwm_manager.hpp"
#include "i2c.h"
#include "STM32_DPS368/util/dps_config.h"

static uint8_t icm_spi_write(uint8_t reg_addr, uint8_t* tx_buffer, uint8_t len) {

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, 0x69 << 1, (uint16_t)reg_addr, I2C_MEMADD_SIZE_8BIT, tx_buffer, len, 100);

    return (status == HAL_OK) ? 0 : 1;
}

static uint8_t icm_spi_read(uint8_t reg_addr, uint8_t* rx_buffer, uint8_t len) {

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, 0x69 << 1, (uint16_t)reg_addr, I2C_MEMADD_SIZE_8BIT, rx_buffer, len, 100);

    return (status == HAL_OK) ? 0 : 1;
}

static void icm_log(const char* msg) {

    printf("[ICM42688P] %s\n", msg);
}


StateError InitState::init(StateContext& context) {

    return StateError::NONE;
}


StateError InitState::update(StateContext& context) {

    if(initialized_){

        printf("[InitState] Already initialized\n");
        return StateError::NONE;
    }

    context.imu.emplace(icm_spi_write, icm_spi_read, icm_log);

    if(context.imu->Connection()){

        printf("ICM42688p Not Found\n");
        return StateError::UPDATE_FAILED_CRITICAL;
    }

    if(context.imu->AccelConfig(::ICM42688P::ACCEL_Mode::LowNoize, ::ICM42688P::ACCEL_SCALE::SCALE02g, ::ICM42688P::ACCEL_ODR::ODR00100hz, ::ICM42688P::ACCEL_DLPF::ODR40) != 0){

        printf("ICM42688p Accel Config Failed\n");
        return StateError::UPDATE_FAILED_CRITICAL;
    }

    if(context.imu->GyroConfig(::ICM42688P::GYRO_MODE::LowNoize, ::ICM42688P::GYRO_SCALE::Dps0250, ::ICM42688P::GYRO_ODR::ODR00100hz, ::ICM42688P::GYRO_DLPF::ODR40) != 0){

        printf("ICM42688p Gyro Config Failed\n");
        return StateError::UPDATE_FAILED_CRITICAL;
    }

    context.baro.emplace();
    context.baro->begin(&hi2c1);

    if(context.baro->startMeasureBothCont(
            DPS__MEASUREMENT_RATE_16,
            DPS__OVERSAMPLING_RATE_1,
            DPS__MEASUREMENT_RATE_64,
            DPS__OVERSAMPLING_RATE_4) != DPS__SUCCEEDED){

        printf("DPS368 Init Failed\n");
        return StateError::UPDATE_FAILED_CRITICAL;
    }

    constexpr float loop_time_s = SystemConfig::MAIN_LOOP_PERIOD_S;

    context.navigation_ekf.emplace();
    context.navigation_ekf->Init(loop_time_s, NavigationEkfConfig::CONFIG);
    context.altitude_data.fill(0.0f);

    context.pwm_manager = std::make_unique<DualcopterPwmManager>();
    context.cascade_pid_manager = std::make_unique<CascadePIDManager>(loop_time_s);

    initialized_ = true;
    return StateError::NONE;
}


StateResult InitState::evaluateNextState(StateContext& context) {

    return {StateChange::STATE_CHANGE, StateID::CALIBRATION};
}


StateID InitState::getStateID() const {

    return StateID::INIT;
}
