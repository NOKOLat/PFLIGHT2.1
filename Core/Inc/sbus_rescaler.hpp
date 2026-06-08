#ifndef SBUS_RESCALER_HPP
#define SBUS_RESCALER_HPP

#include <cstdint>
#include <array>
#include "../Config/sbus_config.hpp"

// ===== 3段階スイッチの状態定義 =====
// LOW=0 が無効（OFF）、MID/HIGH が有効（ON）
enum class SwitchPosition : uint8_t {
    LOW  = 0,  // スイッチ下位（無効 / OFF）
    MID  = 1,  // スイッチ中位（有効 / ON）
    HIGH = 2,  // スイッチ上位（有効 / ON）
};

namespace nokolat {

// ===== リスケーリング後のSBUSデータ構造体 =====
// SBUSの生データを制御用の値に変換した構造体
struct RescaledSBUSData {

    // アナログチャネル
    float throttle;  // スロットル [0~100] %
    float pitch;     // ピッチ [-100~100] %
    float roll;      // ロール [-100~100] %
    float yaw;       // ヨー [-100~100] %

    // AUXチャネル
    SwitchPosition arm;             // Arm [LOW / MID / HIGH]
    SwitchPosition auto_mission;    // 自動離着陸用 [LOW / MID / HIGH]
    SwitchPosition safety;          // 安全装置 [LOW:解除 / MID以上:有効]
    SwitchPosition drop;            // 投下装置トリガー [LOW / MID / HIGH]
    SwitchPosition preflight_debug; // プリフライトデバッグ [LOW / MID / HIGH]
    SwitchPosition flight_debug;    // フライトデバッグ [LOW / MID / HIGH]

    // SBUS状態フラグ
    bool failsafe = false;  // SBUSフェイルセーフフラグ
    bool framelost = false; // SBUSフレームロストフラグ

    // 生データ (デバッグ・ログ用)
    std::array<uint16_t, 18> raw_data = {}; // SBUS生データ
};

// ===== SBUSチャンネルのインデックス定義 =====
// プロポーショナル送信機のチャンネルマッピング
enum class SBUSChannel : uint8_t {

    THROTTLE        = SbusConfig::CH_THROTTLE,
    PITCH           = SbusConfig::CH_PITCH,
    ROLL            = SbusConfig::CH_ROLL,
    YAW             = SbusConfig::CH_YAW,
    DROP            = SbusConfig::CH_DROP,
    ARM             = SbusConfig::CH_ARM,
    SAFETY          = SbusConfig::CH_SAFETY,
    AUTO_MISSION    = SbusConfig::CH_AUTO_MISSION,
    PREFLIGHT_DEBUG = SbusConfig::CH_PREFLIGHT_DEBUG,
    FLIGHT_DEBUG    = SbusConfig::CH_FLIGHT_DEBUG
};

// ===== SBUSリスケーラークラス =====
// SBUSの生データ（360~1692）を制御用の値にリスケールする静的メソッド群
class SBUSRescaler {
public:
    // ===== SBUS生データの範囲定数 =====
    static constexpr uint16_t SBUS_MIN = SbusConfig::SBUS_MIN;
    static constexpr uint16_t SBUS_MID = SbusConfig::SBUS_MID;
    static constexpr uint16_t SBUS_MAX = SbusConfig::SBUS_MAX;

    // ===== 軸ごとのキャリブレーション値構造体 =====
    struct AxisCalib {
        uint16_t min;     // SBUS最小値
        uint16_t center;  // SBUS中立値
        uint16_t max;     // SBUS最大値
    };

    // ===== 閾値設定用構造体 =====
    struct Thresholds {

        // スロットル: 0~100の範囲にマッピング
        uint16_t throttle_min = SbusConfig::THROTTLE_MIN;
        uint16_t throttle_max = SbusConfig::THROTTLE_MAX;

        // 各軸のキャリブレーション: -100~100の範囲にマッピング
        AxisCalib pitch = {SbusConfig::PITCH_MIN, SbusConfig::PITCH_MID, SbusConfig::PITCH_MAX};
        AxisCalib roll  = {SbusConfig::ROLL_MIN,  SbusConfig::ROLL_MID,  SbusConfig::ROLL_MAX};
        AxisCalib yaw   = {SbusConfig::YAW_MIN,   SbusConfig::YAW_MID,   SbusConfig::YAW_MAX};

        // 3段階スイッチの閾値 (0~750: LOW / 751~1500: MID / 1501~2047: HIGH)
        uint16_t switch_low_threshold  = SbusConfig::SWITCH_LOW_THRESHOLD;   // LOW/MID境界
        uint16_t switch_high_threshold = SbusConfig::SWITCH_HIGH_THRESHOLD;  // MID/HIGH境界
    };

    // デフォルト閾値
    static Thresholds default_thresholds;

    // ===== スロットル変換メソッド =====
    // SBUS値を0~100の範囲にリスケール
    static float rescaleThrottle(uint16_t sbus_value,
                                  const Thresholds& thresholds = default_thresholds);

    // ===== 制御入力変換メソッド =====
    // SBUS値を-100~100の範囲にリスケール（ロール、ピッチ、ヨー用）
    static float rescaleControl(uint16_t sbus_value, const AxisCalib& calib);

    // ===== 3段階スイッチ変換メソッド =====
    // SBUS値をSwitchPosition（LOW/MID/HIGH）に変換
    static SwitchPosition rescaleSwitch(uint16_t sbus_value,
                                         const Thresholds& thresholds = default_thresholds);

    // SBUS値を0/1/2に変換（uint8_t版）
    static uint8_t rescaleSwitchInt(uint16_t sbus_value,
                                     const Thresholds& thresholds = default_thresholds);

    // ===== 配列からチャンネルを指定して変換 =====
    // スロットル取得（0~100）
    static float getThrottle(const std::array<uint16_t, 18>& sbus_data,
                             SBUSChannel channel = SBUSChannel::THROTTLE,
                             const Thresholds& thresholds = default_thresholds);

    // 制御入力取得（-100~100）
    static float getControl(const std::array<uint16_t, 18>& sbus_data,
                            SBUSChannel channel,
                            const AxisCalib& calib);

    // 3段階スイッチ取得（SwitchPosition）
    static SwitchPosition getSwitch(const std::array<uint16_t, 18>& sbus_data,
                                     SBUSChannel channel = SBUSChannel::DROP,
                                     const Thresholds& thresholds = default_thresholds);

    // 3段階スイッチ取得（uint8_t版）
    static uint8_t getSwitchInt(const std::array<uint16_t, 18>& sbus_data,
                                 SBUSChannel channel = SBUSChannel::DROP,
                                 const Thresholds& thresholds = default_thresholds);

    // ===== 一括リスケーリングメソッド =====
    // SBUS生データから RescaledSBUSData を生成
    static RescaledSBUSData rescale(const std::array<uint16_t, 18>& sbus_data,
                                     const Thresholds& thresholds = default_thresholds);

    // ===== SBUS値 → 角度 変換 =====
    // mid を 0° 基準として、SBUS生値を角度 [deg] に変換する
    // 手動操縦と同じ変換ロジックを明示化したユーティリティ
    // 例: AILERON_MID(1024) → 0°, AILERON_MAX(1680) → +max_angle_deg
    static float sbusToAngle(uint16_t sbus_value,
                              const AxisCalib& calib,
                              float max_angle_deg = 90.0f);

    // ===== SBUS値 → 角速度 変換 =====
    // mid を 0deg/s 基準として、SBUS生値を角速度 [deg/s] に変換する
    static float sbusToRate(uint16_t sbus_value,
                             const AxisCalib& calib,
                             float max_rate_deg_per_sec);

    // ===== サブトリム角度 計算 =====
    // calib.center（プロポのサブトリム反映値）が標準中心(SBUS_MID)から
    // どれだけずれているかを角度 [deg] に変換して返す
    // 変換式: (center - SBUS_MID) × (180° / (max - min))
    // 自動操縦出力: servo_angle = pid_result + calcSubtrimAngle(calib)
    static float calcSubtrimAngle(const AxisCalib& calib);

    // ===== ユーティリティメソッド =====
    // デッドバンド適用（小さな値を0にする）
    static float applyDeadband(float value, float deadband = 5.0f);

    // 値をクランプ
    static float clamp(float value, float min_val, float max_val);

private:

    // 線形補間
    static float linearMap(uint16_t value, uint16_t in_min, uint16_t in_max,
                           float out_min, float out_max);
};

} // namespace nokolat

#endif // SBUS_RESCALER_HPP
