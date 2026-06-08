#include "../state_headers.hpp"
#include "../../../Config/system_config.hpp"
#include "../Config/sensor_config.hpp"
#include "../../../Utility/PwmManager/dualcopter_pwm_manager.hpp"
#include "i2c.h"

// icm42688p用のSPI書き込み関数
static uint8_t icm_spi_write(uint8_t reg_addr, uint8_t* tx_buffer, uint8_t len) {

	HAL_I2C_Mem_Write(&hi2c1, 0x69 << 1, (uint16_t)reg_addr, I2C_MEMADD_SIZE_8BIT, tx_buffer, len, 100);
    return 0;
}

// icm42688p用のSPI読み取り関数
static uint8_t icm_spi_read(uint8_t reg_addr, uint8_t* rx_buffer, uint8_t len) {

	HAL_I2C_Mem_Read(&hi2c1, 0x69 << 1, (uint16_t)reg_addr, I2C_MEMADD_SIZE_8BIT, rx_buffer, len, 100);
	return 0;
}

// icm42688p用のログ関数
static void icm_log(const char* msg) {

    printf("[ICM42688P] %s\n", msg);
}


StateError InitState::init(StateContext& context) {

    return StateError::NONE;
}


StateError InitState::update(StateContext& context) {

    if (initialized_) {

        printf("[InitState] Already initialized\n");
        return StateError::NONE;
    }

    // icm42688pの初期化
    context.imu.emplace(icm_spi_write, icm_spi_read, icm_log);

    // 通信チェック
    if (context.imu->Connection()) {

        printf("ICM42688p Not Found\n");
        return StateError::UPDATE_FAILED_CRITICAL;
    }

    // センサーの設定
    if (context.imu->AccelConfig(::ICM42688P::ACCEL_Mode::LowNoize, ::ICM42688P::ACCEL_SCALE::SCALE02g, ::ICM42688P::ACCEL_ODR::ODR00100hz, ::ICM42688P::ACCEL_DLPF::ODR40) != 0) {

        printf("ICM42688p Accel Config Failed\n");
        return StateError::UPDATE_FAILED_CRITICAL;
    }

    if (context.imu->GyroConfig(::ICM42688P::GYRO_MODE::LowNoize, ::ICM42688P::GYRO_SCALE::Dps0250, ::ICM42688P::GYRO_ODR::ODR00100hz, ::ICM42688P::GYRO_DLPF::ODR40) != 0) {

        printf("ICM42688p Gyro Config Failed\n");
        return StateError::UPDATE_FAILED_CRITICAL;
    }

    constexpr float loop_time_s = SystemConfig::MAIN_LOOP_PERIOD_S;

    // EKF 遅延初期化
    context.ekf.emplace();
    if (!AttitudeEKF_Init(&context.ekf.value(), loop_time_s)) {

        printf("EKF Init Failed\n");
        return StateError::UPDATE_FAILED_CRITICAL;
    }

    // PwmManager 初期化
    context.pwm_manager = std::make_unique<DualcopterPwmManager>();

    // CascadePIDManager 初期化（FlightStateで使用）
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
