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
    const unsigned long loop_interval_us = loop_manager->getLoopTime();

    if (!loop_manager->isWaitNextLoop()) {

        if (just_ran) {
            printf("[WARN] update() overrun: did not complete within %lu us\r\n", loop_interval_us);
        }

        just_ran = true;
        state_manager.update();

    } else {
        just_ran = false;
    }

}

// UART アイドルライン受信コールバック（ISRManager に委譲）
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {

    ISRManager::handleUartRxEvent(huart, Size);
}
