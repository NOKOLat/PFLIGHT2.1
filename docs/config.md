# Config 一覧

このプロジェクトでユーザーが調整しやすい設定は、主に `Core/Config` 配下のヘッダにまとまっています。値は実行時設定ではなく、ビルド時に反映される `constexpr` / `#define` です。変更後は再ビルドしてください。

## 変更できる config と使用先

| ファイル | 変更できる内容 | 現在値 / 例 | 主な使用先 |
|---|---|---|---|
| `Core/Config/system_config_values.h` | メインループ周期 | `10 ms`, `10000 us`, `0.01 s`, `100 Hz` | `LoopManager` の周期制御、`InitState` の Navigation EKF 初期化、`CascadePIDManager` の `dt` |
| `Core/Config/navigation_ekf_config.hpp` | Navigation EKFのQ/R、校正条件、観測ゲート、気圧有効範囲 | `q_accel_z_bias=3.0e-6`, `r_baro=2.5e-1`, 校正100サンプルなど | `InitState`から`NavigationEKF::Init()`の設定構造体として渡す |
| `Core/Config/sbus_config.hpp` | SBUS チャンネル割り当て | throttle=2, pitch=0, roll=1, yaw=3, drop=4, arm=5, safety=6, auto=7, debug=8/9 | `Core/Inc/sbus_rescaler.hpp` の `SBUSChannel` と `rescale()` |
| `Core/Config/sbus_config.hpp` | SBUS 入力範囲・スイッチ閾値 | min=360, mid=1024, max=1696, low=500, high=1500 | スロットル 0-100%、pitch/roll/yaw -100-100%、3段スイッチ LOW/MID/HIGH への変換 |
| `Core/Config/sbus_config.hpp` | プロポの trim / subtrim | 各軸 0 | `THROTTLE_MIN/MAX`, `PITCH/ROLL/YAW_MIN/MID/MAX` に反映され、SBUS 変換の中心・端点を補正 |
| `Core/Config/cascade_pid_config.hpp` | 指令上限 | pitch/roll 最大角 30 deg、yaw 最大角速度 45 deg/s | `FlightState` で SBUS 入力を目標角度・目標 yaw rate に変換 |
| `Core/Config/cascade_pid_config.hpp` | カスケード PID ゲイン | pitch/roll angle KP=0.9、rate KP=0.7、yaw rate KP=0.275 など | `CascadePIDManager` の各 PID 初期値 |
| `Core/Config/pwm_config.hpp` | モーター・サーボ PWM パルス幅 | motor 1000-2000 us、servo 1000-2000 us | `QuadcopterPwmManager` / `DualcopterPwmManager` の `setPulseRange()` |
| `Core/Config/quadcopter_config.hpp` | クアッド機のモーター出力先 | motor0-3 = `htim3` CH1-4 | `QuadcopterPwmManager` の PWM チャンネル初期化 |
| `Core/Config/dualcopter_config.hpp` | デュアル機のモーター・サーボ出力先 | motor0-1 = `htim12` CH1-2、servo0-3 = `htim1` CH1-4 | `DualcopterPwmManager` の PWM チャンネル初期化。現状 `InitState` は Dualcopter を生成 |
| `Core/Config/sensor_config.hpp` | ICM42688P 用 SPI/CS ピン設定 | SPI=`hspi1`, CS=`GPIOA`, pin=`GPIO_PIN_4` | 現状は `InitState` で include されているが、実際の通信は `hi2c1` / I2C アドレス `0x69` の直書き。変更しても現コードには効かない |

## 特に注意する点

- `system_config_values.h` の周期を変える場合は、`MS` / `US` / `S` / `HZ` の整合を必ずそろえます。ループ周期、EKF の `dt`、PID の `dt` が同じ設定から決まります。
- Navigation EKFの調整値はsubmodule内を直接変更せず、`Core/Config/navigation_ekf_config.hpp`の`NavigationEkfConfig::CONFIG`を変更します。
- SBUS のチャンネル番号は `raw_data[index]` の参照先を変えます。プロポ側の割り当てと一致していないと、スロットルや ARM スイッチが別入力として扱われます。
- `Core/Config/cascade_pid_config.hpp` と `Core/Utility/CascadePID/cascade_pid_config.hpp` は同名ですが、実際に `CascadePIDManager` が include しているのは `Core/Config/cascade_pid_config.hpp` です。調整するなら `Core/Config` 側を変更します。
- `sensor_config.hpp` は現在の初期化処理に実質未接続です。センサー通信ピンを設定で切り替えたい場合は、`InitState` の `icm_spi_write/read` 側も `SensorConfig` を使うように直す必要があります。
