/*
 * konfig.h - IMU_EKF ライブラリ設定ファイル
 *
 * このファイルは Core/Inc に置くことで main リポジトリで管理します
 * （サブモジュール側では konfig.h が .gitignore されているため）
 */
#ifndef KONFIG_H
#define KONFIG_H

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "../Config/system_config_values.h"


/* ===== 状態空間の次元 ===== */
#define SS_X_LEN    (3)     /* 状態: roll, pitch, yaw */
#define SS_Z_LEN    (3)     /* 観測: 加速度センサ 3軸 */
#define SS_U_LEN    (3)     /* 入力: ジャイロ角速度 p, q, r */

/* ===== サンプリング時間 ===== */
/* FlightStateBase::update() の呼び出し周期に合わせてください */
#define SS_DT_MS    (SYSTEM_MAIN_LOOP_PERIOD_MS)   /* 10 ms = 100 Hz */
#define SS_DT       (SYSTEM_MAIN_LOOP_PERIOD_S)


/* ===== 行列サイズ ===== */
/* 使用する最大行列サイズに合わせること（状態次元 = 3） */
#define MATRIX_MAXIMUM_SIZE     (3)

/* 行列の境界チェックを有効にする */
#define MATRIX_USE_BOUNDS_CHECKING


/* ===== 浮動小数点精度 ===== */
#define PRECISION_SINGLE    1
#define PRECISION_DOUBLE    2
#define FPU_PRECISION       (PRECISION_SINGLE)  /* STM32F7 は FPU 搭載、単精度を使用 */

#if (FPU_PRECISION == PRECISION_SINGLE)
    #define float_prec          float
    #define float_prec_ZERO     (1e-7f)
    #define float_prec_ZERO_ECO (1e-5f)
#elif (FPU_PRECISION == PRECISION_DOUBLE)
    #define float_prec          double
    #define float_prec_ZERO     (1e-13)
    #define float_prec_ZERO_ECO (1e-8)
#else
    #error "FPU_PRECISION has not been defined!"
#endif


/* ===== システム実装の選択 ===== */
#define SYSTEM_IMPLEMENTATION_EMBEDDED_NO_PRINT     5
#define SYSTEM_IMPLEMENTATION   (SYSTEM_IMPLEMENTATION_EMBEDDED_NO_PRINT)


/* ===== エラーハンドラ ===== */
/* 実装は flight_state_base.cpp に記述（no-op stub） */
#ifdef __cplusplus
extern "C" {
#endif
void SPEW_THE_ERROR(const char* str);
#ifdef __cplusplus
}
#endif

#define ASSERT(truth, str) { if (!(truth)) SPEW_THE_ERROR(str); }


#endif // KONFIG_H
