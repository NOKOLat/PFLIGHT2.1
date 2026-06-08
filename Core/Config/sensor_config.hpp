#ifndef SENSOR_CONFIG_HPP
#define SENSOR_CONFIG_HPP

#include "stm32f7xx_hal.h"
#include "spi.h"
#include "gpio.h"

namespace SensorConfig {

    namespace ICM42688P {

        // ===== SPI設定 =====
        static inline SPI_HandleTypeDef* const SPI_HANDLE = &hspi1;

        // ===== CS ピン設定 =====
        static GPIO_TypeDef* const     CS_PORT = GPIOA;
        static constexpr uint16_t      CS_PIN  = GPIO_PIN_4;

    } // namespace ICM42688P

} // namespace SensorConfig

#endif // SENSOR_CONFIG_HPP
