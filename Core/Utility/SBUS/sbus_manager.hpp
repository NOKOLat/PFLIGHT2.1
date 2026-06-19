#ifndef SBUS_MANAGER_HPP
#define SBUS_MANAGER_HPP

#include <cstdint>
#include "stm32f7xx_hal.h"
#include "SBUS/sbus.h"
#include "sbus_rescaler.hpp"
#include "isr_manager.hpp"

namespace nokolat {

struct SbusUpdateResult {

    bool is_valid = true;
    bool timeout = false;
    bool failsafe = false;
};

class SbusManager {

    public:

        static constexpr uint32_t TIMEOUT_MS = 50U;

        static void begin(SBUS& receiver, UART_HandleTypeDef* huart) {

            ISRManager::registerSBUS(&receiver, huart);
        }

        static SbusUpdateResult update(SBUS& receiver, RescaledSBUSData& output) {

            const SBUS_DATA& sbus_data = receiver.getData();

            output = SBUSRescaler::rescale(sbus_data.data);
            output.failsafe = sbus_data.failsafe;
            output.framelost = sbus_data.framelost;
            output.raw_data = sbus_data.data;

            SbusUpdateResult result = {};
            const uint32_t now_tick = HAL_GetTick();
            const uint32_t last_valid_frame_tick = ISRManager::getLastValidFrameTick();

            if((now_tick - last_valid_frame_tick) >= TIMEOUT_MS) {

                output.failsafe = true;
                output.framelost = true;

                result.is_valid = false;
                result.timeout = true;
                result.failsafe = true;

                return result;
            }

            if(output.failsafe) {

                result.is_valid = false;
                result.failsafe = true;

                return result;
            }

            return result;
        }
};

} // namespace nokolat

#endif // SBUS_MANAGER_HPP
