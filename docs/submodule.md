# サブモジュール一覧

このドキュメントは、`Core/Lib` 配下に配置されている外部ライブラリと、このプログラム内での役割をまとめたものです。

URL と branch 情報は `.gitmodules` を基準にしています。現在 `Core/Lib` 直下に実体があるサブモジュールは 5 件です。

## 一覧

| パス | リンク | branch | 現在の checkout | 役割 |
| --- | --- | --- | --- | --- |
| `Core/Lib/SBUS` | [NOKOLat/SBUS](https://github.com/NOKOLat/SBUS.git) | `add_ringbuffer_lib` | `821b6c6` | SBUS フレームの受信データ保持、パース、failsafe / framelost 情報の取得 |
| `Core/Lib/ICM42688P` | [NOKOLat/STM32_ICM42688P](https://github.com/NOKOLat/STM32_ICM42688P.git) | 未指定 | `fa2046f` | ICM42688P 6 軸 IMU の接続確認、設定、キャリブレーション、加速度・ジャイロ取得 |
| `Core/Lib/STM32_Motor-Servo_Driver` | [NOKOLat/STM32_Motor-Servo_Driver](https://github.com/NOKOLat/STM32_Motor-Servo_Driver) | 未指定 | `96c17a7` | STM32 HAL タイマを使った PWM、モータ、サーボ出力制御 |
| `Core/Lib/1DoF_PID` | [NOKOLat/1DoF_PID](https://github.com/NOKOLat/1DoF_PID.git) | `master` | `8183890` | 1 軸 PID 制御器。姿勢制御用の Cascade PID の構成要素 |
| `Core/Lib/STM32_DPS368` | [NOKOLat/STM32_DPS368](https://github.com/NOKOLat/STM32_DPS368.git) | 未指定 | `bfd2016` | DPS368から気圧・温度を取得するセンサードライバ |

## 各サブモジュールの役割

### SBUS

`Core/Lib/SBUS` は、プロポ受信機から来る SBUS フレームを扱うライブラリです。

主な機能は次の通りです。

- 25 byte の SBUS フレーム受信バッファを持つ
- SBUS フレームを 18 ch の `SBUS_DATA` にパースする
- `failsafe` と `framelost` を保持する
- RingBuffer からのパースに対応する
- SBUS 生値を PWM 幅に変換する補助関数を持つ

このプログラムでは `StateContext` が `nokolat::SBUS sbus_receiver` を保持し、`ISRManager` が UART DMA 受信イベントで SBUS にデータを流し込みます。

`StateManager::update()` では `sbus_receiver.getData()` で最新の `SBUS_DATA` を取得し、`SBUSRescaler` で throttle、pitch、roll、yaw、arm、safety、auto mission などの制御用データに変換しています。

### ICM42688P

`Core/Lib/ICM42688P` は、TDK InvenSense ICM42688P 6 軸 IMU を扱うライブラリです。

主な機能は次の通りです。

- 通信関数を外から渡して I2C / SPI などに対応する
- IMU 接続確認を行う
- 加速度センサとジャイロセンサの mode、scale、ODR、DLPF を設定する
- 静止キャリブレーションを行う
- 加速度とジャイロの最新値を取得する

このプログラムでは `InitState` で I2C アクセス関数を渡して `StateContext::imu` を生成しています。その後、加速度は `LowNoize / ±2g / 100 Hz / ODR40`、ジャイロは `LowNoize / ±250 dps / 100 Hz / ODR40` に設定されます。

`CalibrationState` ではデータレディを確認して新しいサンプルだけを静止キャリブレーションへ投入し、`FlightStateBase` では毎周期 `context.imu->GetData()` で加速度・ジャイロを取得します。

### STM32_DPS368

`Core/Lib/STM32_DPS368` は、DPS368から気圧と温度を取得するライブラリです。

主な機能は次の通りです。

- I2C / SPI通信によるセンサー初期化
- 気圧・温度の連続測定
- オーバーサンプリングと測定レートの設定
- 補正済み気圧・温度の取得

このプログラムでは`InitState`で連続測定を開始し、`CalibrationState`と`FlightStateBase`が新しい気圧値を`NavigationEKF`へ渡します。

### STM32_Motor-Servo_Driver

`Core/Lib/STM32_Motor-Servo_Driver` は、STM32 HAL のタイマ PWM を使ってモータとサーボを制御するライブラリです。

含まれる主なクラスは次の通りです。

- `PwmController`: タイマ・チャンネルに対して PWM pulse width を設定する基底クラス
- `MotorController`: speed [%] を PWM pulse width に変換して ESC / モータを制御する
- `ServoController`: angle [deg] を PWM pulse width に変換してサーボを制御する

このプログラムでは `Core/Utility/PwmManager` が `MotorController` と `ServoController` を保持します。

`InitState` で `DualcopterPwmManager` が生成され、`ArmState` で PWM を初期化してサーボ・モータを 0 出力にします。`PreFlightState` では SBUS の pitch / roll / yaw 入力をサーボ角に反映し、`FlightStateBase` では PID 出力と throttle を `PwmManager::mix()` に渡して、モータ・サーボへの出力を行います。

### 1DoF_PID

`Core/Lib/1DoF_PID` は、1 自由度の PID 制御器ライブラリです。

主な API は次の通りです。

- `setGain(p, i, d)`
- `setTime(dt)`
- `setLimit(i_max, d_max)`
- `calc(target, process_value)`
- `getData()`
- `reset()`

このプログラムでは `Core/Utility/CascadePID` の `CascadePIDManager` が内部に複数の `PID` インスタンスを持っています。

pitch / roll は角度 PID と角速度 PID のカスケード構成で使われます。yaw は現在、目標 yaw rate とジャイロ yaw rate を使う rate 制御として使われています。`FlightState` は SBUS 入力から目標角・目標角速度を作り、`CascadePIDManager::calcCascadePIDAllAxes()` で 3 軸の PID 出力を計算します。

## このプログラムでの依存関係

概略のデータフローは次の通りです。

```text
SBUS
  -> SBUSRescaler
  -> StateManager / StateContext
  -> FlightState / PreFlightState

ICM42688P
  -> FlightStateBase

STM32_DPS368
  -> FlightStateBase

ICM42688P + STM32_DPS368
  -> NavigationEKF
  -> StateContext::angle / altitude_data
  -> CascadePIDManager
  -> 1DoF_PID
  -> PwmManager
  -> STM32_Motor-Servo_Driver
  -> Motor / Servo output
```

サブモジュール単体は低レイヤの入出力・演算を担当し、このリポジトリ側の `StateManager`、`PwmManager`、`CascadePIDManager` が機体制御向けに組み合わせています。
