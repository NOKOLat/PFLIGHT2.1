#ifndef STATE_CONTEXT_HPP
#define STATE_CONTEXT_HPP

#include <cstdint>
#include <functional>
#include <optional>
#include <memory>
#include <string>
#include <array>
#include "stm32f7xx_hal.h"
#include "SBUS/sbus.h"
#include "sbus_rescaler.hpp"
#include "usart.h"
#include "gpio.h"
#include "ICM42688P/ICM42688P.h"
#include "IMU_EKF/attitude_ekf.h"
#include "../../Utility/PwmManager/pwm_manager.hpp"
#include "../../Utility/CascadePID/cascade_pid_manager.hpp"

struct AngleData {
    float roll  = 0.0f;  // rad
    float pitch = 0.0f;  // rad
    float yaw   = 0.0f;  // rad
};

struct StateContext {

    std::uint32_t state_change_count = 0;
    std::function<void(const std::string&)> publish_log = [](const std::string&) {};

    // SBUS
    nokolat::SBUS            sbus_receiver;
    UART_HandleTypeDef*      sbus_uart  = &huart5;
    UART_HandleTypeDef*      debug_sbus_uart  = &huart2; // デバッグ用（必要に応じて変更）
    nokolat::RescaledSBUSData sbus_data = {};

    // ICM42688P（遅延初期化: InitStateでemplace）
    std::optional<ICM42688P> imu = std::nullopt;
    std::array<float, 3> accel_data = {};
    std::array<float, 3> gyro_data = {};

    // EKF（遅延初期化: FlightStateBase::initでemplace）
    std::optional<AttitudeEKF_t> ekf = std::nullopt;
    AngleData angle = {};

    // PwmManager（遅延初期化: InitStateでmake_unique）
    std::unique_ptr<PwmManager> pwm_manager = nullptr;

    // CascadePIDManager（遅延初期化: InitStateでmake_unique）
    std::unique_ptr<CascadePIDManager> cascade_pid_manager = nullptr;

    // PID出力（FlightState::onUpdate()で計算）
    std::array<float, 3> pid_output = {};  // [pitch, roll, yaw]
    float throttle = 0.0f;
};


#endif // STATE_CONTEXT_HPP
