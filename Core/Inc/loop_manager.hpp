#ifndef LOOP_MANAGER_HPP
#define LOOP_MANAGER_HPP

#include "core_cm7.h"
#include "stm32f7xx.h"

#include "../Config/system_config.hpp"

class LoopManager {

    public:

        // コンストラクタ
        LoopManager(unsigned long loop_interval_us = SystemConfig::MAIN_LOOP_PERIOD_US) {

            this->loop_interval_us = loop_interval_us;
            this->last_cycle_count = 0;
            
            // DWT (Data Watchpoint and Trace) の初期化
            CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
            DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        }

        // 次ループまで待機が必要かどうかを判定
        bool isWaitNextLoop() {

            uint32_t current_cycle_count = DWT->CYCCNT;

            // 最初の実行時は待機しない
            if (last_cycle_count == 0) {

                last_cycle_count = current_cycle_count;
                return false;
            }

            // サイクルカウントからマイクロ秒を計算
            uint32_t delta_cycles = current_cycle_count - last_cycle_count;
            unsigned long delta_time_us = delta_cycles / (SystemCoreClock / 1000000);

            // 指定時間経過しているかの判定
            if (delta_time_us >= loop_interval_us) {

                last_cycle_count = current_cycle_count;
                return false;
            }

            // 指定時間経過していない場合は待機
            return true;
        }

        // ループタイムの取得
        unsigned long getLoopTime() const {
            return loop_interval_us;
        }

    private:

        // ループ間隔（マイクロ秒）
        unsigned long loop_interval_us = SystemConfig::MAIN_LOOP_PERIOD_US;

        // 最後のDWTサイクルカウント
        uint32_t last_cycle_count = 0;
};

#endif // LOOP_MANAGER_HPP
