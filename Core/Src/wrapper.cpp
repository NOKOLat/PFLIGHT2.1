#include "wrapper.hpp"
#include <optional>

#include "StateManager/state_manager.hpp"
#include "loop_manager.hpp"
#include "isr_manager.hpp"
#include "usart.h"

StateManager state_manager(StateID::INIT);
std::optional<LoopManager> loop_manager;

void init(){

    printf("Build: 1\n");
    loop_manager.emplace();
    state_manager.init();
}

void loop(){

    static bool just_ran = false;
    static unsigned long accumulated_loop_time_us = 0;
    static uint32_t measured_loop_count = 0;
    constexpr uint32_t average_loop_count = 100;
    const unsigned long loop_interval_us = loop_manager->getLoopTime();

    if (!loop_manager->isWaitNextLoop()) {

        if (just_ran) {
            printf("[WARN] update() overrun: did not complete within %lu us\r\n", loop_interval_us);
        }

        just_ran = true;

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

    } else {
        just_ran = false;
    }

}

// UART アイドルライン受信コールバック（ISRManager に委譲）
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {

    ISRManager::handleUartRxEvent(huart, Size);
}
