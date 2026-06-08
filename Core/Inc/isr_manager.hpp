#ifndef ISR_MANAGER_HPP
#define ISR_MANAGER_HPP

#include "stm32f7xx_hal.h"
#include "SBUS/sbus.h"

// 割り込み処理とアプリケーションロジックを仲介する静的クラス
namespace ISRManager {

    // SBUSインスタンスを登録する
    // sbus: データを流し込む対象のSBUSインスタンス
    // huart: SBUSが接続されているUARTハンドル
    void registerSBUS(nokolat::SBUS* sbus, UART_HandleTypeDef* huart);

    // UART受信イベントハンドラ（アイドルライン検出版）
    // HAL_UARTEx_RxEventCallback から呼び出されることを想定
    void handleUartRxEvent(UART_HandleTypeDef* huart, uint16_t size);

    // 最後に有効なSBUSフレームを受信した時刻を返す [ms]
    // HAL_GetTick() との差分でタイムアウト判定に使用する
    uint32_t getLastValidFrameTick();
}

#endif // ISR_MANAGER_HPP
