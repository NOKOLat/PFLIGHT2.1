#ifndef STATE_CONTEXT_HPP
#define STATE_CONTEXT_HPP

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include "stm32f7xx_hal.h"
#include "SBUS/sbus.h"
#include "SBUS/sbus_rescaler.hpp"
#include "usart.h"
#include "gpio.h"
#include "ICM42688P/ICM42688P.h"
#include "STM32_DPS368/Dps3xx.h"
#include "Navigation_EKF/navigation_ekf.h"
#include "../../Utility/PwmManager/pwm_manager.hpp"
#include "../../Utility/CascadePID/cascade_pid_manager.hpp"

struct AngleData {
    float roll  = 0.0f;  // deg
    float pitch = 0.0f;  // deg
    float yaw   = 0.0f;  // deg
};

struct StateContext {

    std::uint32_t state_change_count = 0;
    std::function<void(const std::string&)> publish_log = [](const std::string&) {};

    // SBUS
    nokolat::SBUS sbus_receiver;
    UART_HandleTypeDef* sbus_uart = &huart5;
    UART_HandleTypeDef* debug_sbus_uart = &huart2;
    nokolat::RescaledSBUSData sbus_data = {};

    // ICM42688P
    std::optional<ICM42688P> imu = std::nullopt;
    std::array<float, 3> accel_data = {};
    std::array<float, 3> gyro_data = {};

    // DPS368 pressure sensor
    std::optional<Dps3xx> baro = std::nullopt;
    float pressure_pa = 0.0f;
    float temperature_c = 0.0f;

    // Navigation EKF
    std::optional<NavigationEKF> navigation_ekf = std::nullopt;
    std::array<float, 3> altitude_data = {};  // [altitude(m), velocity(m/s), accel(m/s^2)]
    AngleData angle = {};

    // PWM and control
    std::unique_ptr<PwmManager> pwm_manager = nullptr;
    std::unique_ptr<CascadePIDManager> cascade_pid_manager = nullptr;
    std::array<float, 3> pid_output = {};  // [pitch, roll, yaw]
    float throttle = 0.0f;
};

#endif // STATE_CONTEXT_HPP
