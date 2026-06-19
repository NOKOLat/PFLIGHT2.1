#include "wrapper.hpp"
#include <optional>

#include "StateManager/state_manager.hpp"
#include "loop_manager.hpp"
#include "SBUS/isr_manager.hpp"
#include "usart.h"

StateManager state_manager(StateID::INIT);
std::optional<LoopManager> loop_manager;

void init(){

    printf("Build: 1\n");
    loop_manager.emplace();
    state_manager.init();
}

void loop(){

    static unsigned long accumulated_loop_time_us = 0;
    static uint32_t measured_loop_count = 0;
    constexpr uint32_t average_loop_count = 100;

    if (!loop_manager->isWaitNextLoop()) {

        const uint32_t update_start_cycle = DWT->CYCCNT;
        state_manager.update();
        const uint32_t update_end_cycle = DWT->CYCCNT;
        const uint32_t update_delta_cycles = update_end_cycle - update_start_cycle;
        const unsigned long update_time_us = update_delta_cycles / (SystemCoreClock / 1000000);

        accumulated_loop_time_us += update_time_us;
        measured_loop_count ++;
        if(measured_loop_count >= average_loop_count){

            const unsigned long average_loop_time_us = accumulated_loop_time_us / measured_loop_count;
            printf("[INFO] average update loop time: %lu us / %lu loops\r\n", average_loop_time_us, (unsigned long)measured_loop_count);

            accumulated_loop_time_us = 0;
            measured_loop_count = 0;
        }
    }

}

// UART アイドルライン受信コールバック（ISRManager に委譲）
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {

    ISRManager::handleUartRxEvent(huart, Size);
}
