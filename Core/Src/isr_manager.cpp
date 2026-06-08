#include "isr_manager.hpp"
#include <stdio.h>

namespace ISRManager {

    // 登録されたSBUSインスタンス
    static nokolat::SBUS* sbus_instance = nullptr;
    static UART_HandleTypeDef* sbus_huart = nullptr;

    // DMA受信用バッファ（25バイト = 1フレーム）
    static constexpr uint8_t SBUS_FRAME_SIZE = 25;
    static uint8_t sbus_receive_buffer[SBUS_FRAME_SIZE];

    // 最後に有効なフレームを受信した時刻 [ms]
    static uint32_t last_valid_frame_tick = 0;

    void registerSBUS(nokolat::SBUS* sbus, UART_HandleTypeDef* huart) {

        sbus_instance = sbus;
        sbus_huart = huart;

        if (sbus_instance && sbus_huart) {

            // 初期タイムスタンプを現在時刻に設定（初回タイムアウトを防ぐ）
            last_valid_frame_tick = HAL_GetTick();

            // DMA受信開始（ReceiveToIdle版）
            HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(sbus_huart, sbus_receive_buffer, SBUS_FRAME_SIZE);
            printf("[ISRManager] SBUS DMA start: %s\n", (status == HAL_OK) ? "OK" : "FAILED");
        }
    }

    void handleUartRxEvent(UART_HandleTypeDef* huart, uint16_t size) {

        // 登録されたUARTハンドルと一致し、かつ受信サイズが正当な場合のみ処理
        if (sbus_instance && sbus_huart == huart && size == SBUS_FRAME_SIZE) {

            // 受信したデータをSBUSバッファにコピー
            uint8_t* sbus_buffer = sbus_instance->getReceiveBufferPtr();
            for (uint8_t i = 0; i < SBUS_FRAME_SIZE; i++) {
                sbus_buffer[i] = sbus_receive_buffer[i];
            }

            // 受信タイムスタンプを更新
            last_valid_frame_tick = HAL_GetTick();

            // パース要求フラグを立てる
            sbus_instance->requireParse(true);

            // 次の25バイトをアイドルライン検出で受信
            HAL_UARTEx_ReceiveToIdle_DMA(sbus_huart, sbus_receive_buffer, SBUS_FRAME_SIZE);
        }
    }

    uint32_t getLastValidFrameTick() {

        return last_valid_frame_tick;
    }
}
