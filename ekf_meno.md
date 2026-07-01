# IMU + 気圧 統合EKF 再設計計画

## Summary
- 既存の「姿勢EKF + 高度KF」のカスケードをやめ、`IMU + 気圧` を1つの統合EKFで推定する。
- 旧 `Altitude` / `AttitudeEKF_t` は削除し、`NavigationEKF` に一本化する。
- 新EKFは内部をrad/quaternionで持ち、既存PID互換のため `StateContext::angle` にはdeg、`altitude_data` には `[高度m, 鉛直速度m/s, 鉛直加速度m/s^2]` を出す。

## Key Changes
- 新規ライブラリを `Core/Lib/Navigation_EKF` に追加する。
  - 公開APIは `NavigationEKF::Init(dt)`, `CalibrateSample(accel, gyro, pressure)`, `IsCalibrated()`, `Update(accel, gyro_rad, pressure)`, `GetAnglesDeg(out3)`, `GetAltitudeData(out3)` とする。
  - 専用固定配列の小型行列演算を同ライブラリ内に持ち、外部の行列ライブラリには依存しない。
- 誤差状態は以下の9次元で固定する。
  - `姿勢誤差3軸,vz,z,bgx,bgy,bgz,baz`
  - EKF更新対象: quaternionは更新後に必ず正規化する。
  - yawは絶対補正なしのgyro積分として出力する。
- センサモデルは以下に固定する。
  - 入力: accel `[m/s^2]`, gyro `[rad/s]`, pressure `[Pa]`
  - 予測: gyro biasを引いてquaternion積分、`R(q) * [ax, ay, az - baz] - g` から鉛直加速度を作り、`vz,z` を積分。
  - 観測: 正規化accelの重力方向3軸 + 気圧高度1軸。
  - 気圧高度はキャリブレーション平均圧力を0m基準として `44330 * (1 - pow(p / p0, 0.1903))` で変換する。
- 加速度観測は動的ゲートを入れる。
  - `abs(norm(accel) - 9.80665)` が小さい時は通常R。
  - 閾値を超える時は加速度観測Rを大きくして、飛行加速度でroll/pitchを壊しにくくする。
- 気圧観測は毎回有効。ただしpressureが0以下/NaNなら気圧更新をスキップして予測のみ継続。
  - DPS368が新しいpressureを返した周期だけ気圧更新し、古い値を再利用しない。
  - 仮実装では直近10ループの有効なpressureをリングバッファで移動平均してから高度へ変換する。
  - 直前の気圧移動平均高度との差が10 mを超える単発の気圧高度は外れ値として棄却する。
- `StateContext` は `std::optional<NavigationEKF> navigation_ekf` に置換する。
  - `altitude_estimator` と `ekf` はStateManagerから参照しない。
  - `AngleData` のコメントを実態に合わせてdegへ修正する。
- `InitState` / `CalibrationState` / `FlightStateBase` を新EKFに差し替える。
  - CalibrationStateで100 Hzの実時間に合わせて新しい静止サンプル100回を集め、gyro bias、気圧基準、初期roll/pitchを決める。
  - 加速度ノルムまたは角速度が静止条件から外れるサンプルと、新しい気圧値がない周期は校正回数に含めない。
  - FlightStateBaseではIMU/気圧取得後、gyroをdpsからrad/sへ変換して `NavigationEKF::Update` を1回だけ呼ぶ。
  - 出力を `context.angle` と `context.altitude_data` に格納して、既存PIDとPWM処理はそのまま使う。

## Implementation Notes
- EKFは9次元誤差状態、最大観測4次元の固定配列で実装する。
- v1では実装ミスを減らすため、状態遷移ヤコビアンFと観測ヤコビアンHは有限差分で計算する。
- 共分散更新は `P = FPF^T + Q`, `K = PH^T(HPH^T+R)^-1`, `x = x + K residual` とJoseph形式 `P = (I-KH)P(I-KH)^T + KRK^T` を使う。
- 静止時に気圧高度の揺れが鉛直速度へ回り込まない初期調整値として、`Q_VELOCITY = 1.0e-5`, `Q_ALTITUDE = 1.0e-5`, `R_BARO = 2.5e-1` を使う。
- DPS368は温度16 Hz・OSR 1、気圧64 Hz・OSR 4で動作させ、最大レート・OSR 1よりも気圧ノイズを抑える。
- 角度出力はquaternionからroll/pitch/yawを算出し、StateContextへdegで保存する。
- `STM32_AI/CODING_GUIDE.md` の既存スタイルに合わせ、4スペース、関数前Doxygen風コメント、同一行 `{` を使う。

## Test Plan
- 静的確認:
  - `rg` でStateManagerから旧 `Altitude` / `AttitudeEKF_Update` 呼び出しが消えていることを確認。
  - `AngleData` とPID入力の単位がdegで統一されていることを確認。
- ロジック確認:
  - 静止入力 `accel=[0,0,9.81]`, `gyro=0`, `pressure=p0` でroll/pitch/高度/速度が0近傍に保たれる。
  - pressure低下時に高度が正方向へ更新される。
  - `norm(accel)` が大きく外れた時に加速度観測Rが増える。
  - pressure不正値では気圧更新をスキップし、NaNを出さない。
- ビルド:
  - STM32CubeIDE管理プロジェクトとしてDebugビルドを実行する。
  - 既存の `STM32_AI/stm32_build` 手順がこのcheckoutに合わない場合は、CubeIDE生成済み `Debug` ディレクトリまたはIDE headless buildで代替する。
  - ビルドエラーは新EKF差し替え範囲に限って修正する。

## Assumptions
- v1は高度・鉛直速度・roll/pitch安定化を主目的にし、yawの絶対精度向上は対象外。
- 互換性不要だが、旧ライブラリ削除はしない。戻せる余地を残してStateManagerの使用先だけ置き換える。
- キャリブレーション中は機体が静止している前提。
- 既存PIDはdeg入力を期待しているため、外部出力単位はdegを維持する。
