# StateMachine と状態設計

このドキュメントは、`Core/StateManager` を中心に実装されている状態機械と、各状態の責務・遷移条件をまとめたものです。

## 全体像

本プログラムでは、メインループから `StateManager::update()` を周期呼び出しする明示的な有限状態機械を採用しています。

状態ごとの処理は `StateInterface` を実装したクラスとして分離されており、`StateManager` は現在状態のインスタンスを 1 つだけ保持します。状態遷移が発生すると、`StateFactory` が次の `StateID` に対応する状態クラスを生成し、`init()` を呼んだ後に現在状態を差し替えます。

基本的な 1 周期の流れは次の通りです。

1. 初回 `update()` で初期状態 `INIT` を生成する。
2. SBUS 受信値を読み取り、制御用の `RescaledSBUSData` に変換して `StateContext` に反映する。
3. SBUS timeout / failsafe を検出した場合は `EMERGENCY_STOP` 用のフォールバック処理へ移る。
4. 現在状態の `update(context)` を呼ぶ。
5. `update()` が重大エラーを返した場合は `EMERGENCY_STOP` へフォールバックする。
6. 現在状態の `evaluateNextState(context)` で遷移判定を行う。
7. 遷移要求があれば次状態を生成し、`init(context)` に成功した場合のみ状態を差し替える。

## 実行周期

`wrapper.cpp` で `StateManager state_manager(StateID::INIT)` が生成され、`init()` と `loop()` から呼ばれます。

`LoopManager` は `SystemConfig::MAIN_LOOP_PERIOD_US` を使って周期制御します。現在の設定値は `10 ms`、つまり `100 Hz` です。

`loop()` は周期到達時のみ `state_manager.update()` を実行します。前回の `update()` が周期内に終わっていない場合は overrun 警告を出します。

## StateManager

`StateManager` は状態機械の実行主体です。

主な責務は次の通りです。

- 現在状態 `current_state_` の保持
- 初期状態 `init_state_id_` からの起動
- `StateFactory` による状態生成
- 状態ごとの `init()` / `update()` / `evaluateNextState()` 呼び出し
- `StateContext` の保持と各状態への共有
- SBUS 受信値の更新
- SBUS timeout / failsafe / critical error 時の `EMERGENCY_STOP` フォールバック

`StateManager::init()` では `EMERGENCY_STOP` 状態をあらかじめ生成し、SBUS を `ISRManager` に登録します。初期状態 `INIT` の生成は `init()` ではなく、最初の `update()` で行われます。

## StateInterface

すべての状態は `StateInterface` を実装します。

```cpp
virtual StateError  init(StateContext& context) = 0;
virtual StateError  update(StateContext& context) = 0;
virtual StateResult evaluateNextState(StateContext& context) = 0;
virtual StateID     getStateID() const = 0;
```

各関数の役割は次の通りです。

- `init()`: 状態に入った直後に 1 回だけ実行する初期化処理
- `update()`: 状態滞在中に周期実行する処理
- `evaluateNextState()`: 次状態へ遷移するかどうかの判定
- `getStateID()`: 現在状態の識別子を返す

`StateError::UPDATE_FAILED_CRITICAL` が返ると、`StateManager` は通常の状態遷移ではなく `EMERGENCY_STOP` フォールバックへ移行します。

## StateContext

`StateContext` は状態間で共有する実行時コンテキストです。状態クラスはグローバル変数を直接持つのではなく、原則として `StateContext` 経由でセンサ、入力、制御器、出力にアクセスします。

主な内容は次の通りです。

- ログ出力関数 `publish_log`
- SBUS 受信器、UART ハンドル、リスケール済み SBUS データ
- IMU `ICM42688P`
- 加速度・ジャイロの最新値
- 姿勢推定用 EKF
- 推定姿勢角 `roll / pitch / yaw`
- `PwmManager`
- `CascadePIDManager`
- PID 出力と throttle

IMU、EKF、PWM、PID は遅延初期化されます。

現在の実装では `InitState` が IMU / EKF / PWM / CascadePID を初期化します。

## StateFactory

`StateFactory::createState(StateID)` は `StateID` から対応する状態クラスを `std::unique_ptr<StateInterface>` として生成します。

未定義または不正な `StateID` が渡された場合、現在の実装では `EmergencyStopState` を返します。

そのため、生成不能な状態 ID は安全側の停止状態に寄せられます。

## 状態一覧

| StateID | クラス | 主な責務 |
| --- | --- | --- |
| `INIT` | `InitState` | IMU 接続確認、IMU 設定、EKF 初期化、PWM 管理器生成、Cascade PID 管理器生成 |
| `CALIBRATION` | `CalibrationState` | IMU キャリブレーション |
| `PRE_ARM` | `PreArmState` | Arm スイッチ待ち |
| `ARM` | `ArmState` | PWM 初期化、サーボ・モータ出力を 0 にセット |
| `PRE_FLIGHT` | `PreFlightState` | Arm 後、Safety スイッチが入るまでのサーボ追従確認 |
| `FLIGHT` | `FlightState` | 手動飛行制御、姿勢推定、Cascade PID、PWM ミキシング |
| `AUTO_FLIGHT` | `AutoFlightState` | 自動飛行モード枠。現状は共通飛行処理のみで独自制御は未実装 |
| `DIS_ARM` | `DisArmState` | PWM 停止後、`PRE_ARM` へ戻す |
| `MOTOR_SERVO_TEST` | `MotorServoTestState` | サーボ・モータの順次テスト |
| `ERROR` | `ErrorState` | PWM を継続停止し、状態を維持 |
| `EMERGENCY_STOP` | `EmergencyStopState` | StateManager のフォールバック専用。PWM を継続停止 |

## 通常遷移

通常の起動・飛行系の遷移は次の通りです。

```text
INIT
  -> CALIBRATION
  -> PRE_ARM
  -> ARM
  -> PRE_FLIGHT
  -> FLIGHT
```

各遷移条件は次の通りです。

| 現在状態 | 次状態 | 条件 |
| --- | --- | --- |
| `INIT` | `CALIBRATION` | 初期化処理が正常完了した後 |
| `CALIBRATION` | `PRE_ARM` | IMU キャリブレーション実行後 |
| `PRE_ARM` | `ARM` | `sbus_data.arm == HIGH` |
| `ARM` | `PRE_FLIGHT` | PWM 初期化とゼロ出力設定後 |
| `PRE_FLIGHT` | `PRE_ARM` | `sbus_data.arm != HIGH` |
| `PRE_FLIGHT` | `FLIGHT` | `sbus_data.safety == HIGH` |
| `FLIGHT` | `DIS_ARM` | `sbus_data.arm != HIGH` |
| `FLIGHT` | `AUTO_FLIGHT` | `sbus_data.auto_mission == HIGH` |
| `AUTO_FLIGHT` | `DIS_ARM` | `sbus_data.arm != HIGH` |
| `AUTO_FLIGHT` | `FLIGHT` | `sbus_data.auto_mission != HIGH` |
| `DIS_ARM` | `PRE_ARM` | PWM 停止処理後 |
| `MOTOR_SERVO_TEST` | `PRE_ARM` | 全サーボ・全モータのテスト完了後 |
| `ERROR` | `ERROR` | 常に状態維持 |
| `EMERGENCY_STOP` | `EMERGENCY_STOP` | 常に状態維持 |

## FlightStateBase

`FLIGHT` と `AUTO_FLIGHT` は `FlightStateBase` を継承しています。

`FlightStateBase` は飛行中に共通で必要な処理を担当します。

1. IMU から加速度・ジャイロを取得する。
2. ジャイロ値を dps から rad/s に変換する。
3. EKF を更新する。
4. 推定した roll / pitch / yaw を `StateContext::angle` に保存する。
5. 派生クラスの `onUpdate()` を呼び、throttle と PID 出力を更新する。
6. 2 周期に 1 回、`PwmManager::mix()` と `output()` を実行する。

現在のメインループ設定は 100 Hz なので、飛行中の PWM 出力更新はおおむね 50 Hz になります。

`FlightState` は SBUS 入力から目標 pitch / roll 角度、目標 yaw 角速度、throttle を作り、`CascadePIDManager` で PID 出力を計算します。

`AutoFlightState` は `FlightStateBase` の共通処理には乗っていますが、現状の `onUpdate()` は何もしていません。自動飛行の制御ロジックは今後ここに追加する想定です。

## Safety とフォールバック

状態機械全体の安全側処理は `StateManager` が優先的に扱います。

次の場合、通常状態の遷移判定より前に `EMERGENCY_STOP` フォールバックへ移行します。

- SBUS の有効フレームが 50 ms 以上更新されない
- SBUS データの `failsafe` が立っている
- 現在状態の `update()` が `StateError::UPDATE_FAILED_CRITICAL` を返す
- 初期状態または次状態の生成・初期化に失敗する

フォールバック中は `current_state_` ではなく、あらかじめ生成された `fallback_emergency_stop_` の `update()` が呼ばれます。`EmergencyStopState` は `pwm_manager->stop()` を繰り返し呼び、出力を停止し続けます。

## 実装上の注意

- `StateManager` は `EmergencyStopState::update()` が `StateError::CRITICAL_STOPPED` を返した場合に `SHUTDOWN` を返す設計になっています。ただし、現在の `EmergencyStopState::update()` は `StateError::NONE` を返すため、実際には停止出力を継続しながら `CONTINUE` し続けます。
- `MotorServoTestState` の `ONE_SECOND_COUNT` は 400 ですが、現在のメインループは 100 Hz です。そのため、この値をそのまま使うと 1 秒ではなく約 4 秒相当になります。
- `StateFactory` の default は `EmergencyStopState` を生成しますが、`changeState()` 側では生成成功として扱われます。不正な `StateID` を明示的な失敗として扱いたい場合は、Factory の default 方針を見直す必要があります。
- `ERROR` 状態は定義されていますが、現在の `StateManager` は critical error 時に `ERROR` ではなく `EMERGENCY_STOP` フォールバックを使います。
