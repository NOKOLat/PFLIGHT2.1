# サブモジュール一覧

このドキュメントは、`Core/Lib` 配下に配置されている外部ライブラリと、このプログラム内での役割をまとめたものです。

URL と branch 情報は `.gitmodules` を基準にしています。現在 `Core/Lib` 直下に実体があるサブモジュールは 5 件です。

## 一覧

| パス | リンク | branch | 現在の checkout | 役割 |
| --- | --- | --- | --- | --- |
| `Core/Lib/SBUS` | [NOKOLat/SBUS](https://github.com/NOKOLat/SBUS.git) | `add_ringbuffer_lib` | `821b6c6` | SBUS フレームの受信データ保持、パース、failsafe / framelost 情報の取得 |
| `Core/Lib/ICM42688P` | [NOKOLat/STM32_ICM42688P](https://github.com/NOKOLat/STM32_ICM42688P.git) | 未指定 | `f4ccb10` | ICM42688P 6 軸 IMU の接続確認、設定、キャリブレーション、加速度・ジャイロ取得 |
| `Core/Lib/IMU_EKF` | [NOKOLat/IMU_EKF](https://github.com/NOKOLat/IMU_EKF.git) | 未指定 | `d2a3e14` | IMU の加速度・ジャイロから姿勢角を推定する EKF |
| `Core/Lib/STM32_Motor-Servo_Driver` | [NOKOLat/STM32_Motor-Servo_Driver](https://github.com/NOKOLat/STM32_Motor-Servo_Driver) | 未指定 | `96c17a7` | STM32 HAL タイマを使った PWM、モータ、サーボ出力制御 |
| `Core/Lib/1DoF_PID` | [NOKOLat/1DoF_PID](https://github.com/NOKOLat/1DoF_PID.git) | `master` | `8183890` | 1 軸 PID 制御器。姿勢制御用の Cascade PID の構成要素 |

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

`CalibrationState` では `context.imu->Calibration(1000)` を実行し、`FlightStateBase` では毎周期 `context.imu->GetData()` で加速度・ジャイロを取得します。

### IMU_EKF

`Core/Lib/IMU_EKF` は、6 軸 IMU の加速度・ジャイロから姿勢を推定する EKF ライブラリです。

主な機能は次の通りです。

- `AttitudeEKF_t` による姿勢推定状態の保持
- `AttitudeEKF_Init()` による初期化
- `AttitudeEKF_Update()` による推定更新
- roll / pitch / yaw の取得

このプログラムでは `InitState` が `SystemConfig::MAIN_LOOP_PERIOD_S` を使って EKF を初期化します。

`FlightStateBase` では IMU から取得したジャイロ値を deg/s から rad/s に変換し、`AttitudeEKF_Update()` に渡しています。更新後、roll / pitch / yaw を deg に変換して `StateContext::angle` に保存します。

README にも記載がある通り、6 軸 IMU のみを使うため yaw 角の絶対推定はずれやすく、yaw は角速度ベースの制御で使う前提に近い設計です。

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

## .gitmodules 上の注意

`.gitmodules` には次のエントリも残っています。

| パス | リンク | 状態 |
| --- | --- | --- |
| `Core/Lib/Motor_Servo_Driver` | [NOKOLat/STM32_Motor-Servo_Driver](https://github.com/NOKOLat/STM32_Motor-Servo_Driver) | 現在の作業ツリーにはディレクトリが存在しない |

実体があるのは `Core/Lib/STM32_Motor-Servo_Driver` です。`Core/Lib/Motor_Servo_Driver` は旧名称または重複登録の可能性があります。

## このプログラムでの依存関係

概略のデータフローは次の通りです。

```text
SBUS
  -> SBUSRescaler
  -> StateManager / StateContext
  -> FlightState / PreFlightState

ICM42688P
  -> FlightStateBase
  -> IMU_EKF
  -> StateContext::angle
  -> CascadePIDManager
  -> 1DoF_PID
  -> PwmManager
  -> STM32_Motor-Servo_Driver
  -> Motor / Servo output
```

サブモジュール単体は低レイヤの入出力・演算を担当し、このリポジトリ側の `StateManager`、`PwmManager`、`CascadePIDManager` が機体制御向けに組み合わせています。
