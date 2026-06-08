#include "sbus_rescaler.hpp"
#include <algorithm>

namespace nokolat {

// デフォルト閾値の初期化
SBUSRescaler::Thresholds SBUSRescaler::default_thresholds = {};

// ===== 線形補間の実装 =====
float SBUSRescaler::linearMap(uint16_t value, uint16_t in_min, uint16_t in_max,
                               float out_min, float out_max) {

    // 入力値を範囲内にクランプ
    if (value <= in_min) {

        return out_min;
    }

    if (value >= in_max) {

        return out_max;
    }

    // 線形補間
    float normalized = static_cast<float>(value - in_min) / static_cast<float>(in_max - in_min);
    return out_min + normalized * (out_max - out_min);
}

// ===== スロットル変換: SBUS値 -> 0~100 =====
float SBUSRescaler::rescaleThrottle(uint16_t sbus_value, const Thresholds& thresholds) {

    return linearMap(sbus_value, thresholds.throttle_min, thresholds.throttle_max, 0.0f, 100.0f);
}

// ===== 制御入力変換: SBUS値 -> -100~100 =====
float SBUSRescaler::rescaleControl(uint16_t sbus_value, const AxisCalib& calib) {

    if (sbus_value < calib.center) {

        // センター以下: min -> center を -100 -> 0 にマップ
        return linearMap(sbus_value, calib.min, calib.center, -100.0f, 0.0f);
    }
    else {

        // センター以上: center -> max を 0 -> 100 にマップ
        return linearMap(sbus_value, calib.center, calib.max, 0.0f, 100.0f);
    }
}

// ===== 3段階スイッチ変換: SBUS値 -> 0/1/2 =====
SwitchPosition SBUSRescaler::rescaleSwitch(uint16_t sbus_value, const Thresholds& thresholds) {

    if (sbus_value < thresholds.switch_low_threshold) {

        return SwitchPosition::LOW;
    }
    else if (sbus_value < thresholds.switch_high_threshold) {

        return SwitchPosition::MID;
    }
    else {

        return SwitchPosition::HIGH;
    }
}

// ===== 3段階スイッチ変換: SBUS値 -> 0/1/2 (uint8_t版) =====
uint8_t SBUSRescaler::rescaleSwitchInt(uint16_t sbus_value, const Thresholds& thresholds) {

    return static_cast<uint8_t>(rescaleSwitch(sbus_value, thresholds));
}

// ===== 配列からスロットル取得 =====
float SBUSRescaler::getThrottle(const std::array<uint16_t, 18>& sbus_data,
                                 SBUSChannel channel,
                                 const Thresholds& thresholds) {

    uint8_t index = static_cast<uint8_t>(channel);

    if (index >= sbus_data.size()) {

        return 0.0f;  // 範囲外の場合は0を返す
    }

    return rescaleThrottle(sbus_data[index], thresholds);
}

// ===== 配列から制御入力取得 =====
float SBUSRescaler::getControl(const std::array<uint16_t, 18>& sbus_data,
                                SBUSChannel channel,
                                const AxisCalib& calib) {

    uint8_t index = static_cast<uint8_t>(channel);

    if (index >= sbus_data.size()) {

        return 0.0f;  // 範囲外の場合は0を返す
    }

    return rescaleControl(sbus_data[index], calib);
}

// ===== 配列からスイッチ取得 =====
SwitchPosition SBUSRescaler::getSwitch(const std::array<uint16_t, 18>& sbus_data,
                                        SBUSChannel channel,
                                        const Thresholds& thresholds) {

    uint8_t index = static_cast<uint8_t>(channel);

    if (index >= sbus_data.size()) {

        return SwitchPosition::LOW;  // 範囲外の場合はLOWを返す
    }

    return rescaleSwitch(sbus_data[index], thresholds);
}

// ===== 配列からスイッチ取得(uint8_t版) =====
uint8_t SBUSRescaler::getSwitchInt(const std::array<uint16_t, 18>& sbus_data,
                                    SBUSChannel channel,
                                    const Thresholds& thresholds) {

    return static_cast<uint8_t>(getSwitch(sbus_data, channel, thresholds));
}

// ===== SBUS値 → 角度 変換 =====
// mid を 0° 基準として角度 [deg] に変換する
// 変換式: rescaleControl() [-100,100] × (max_angle_deg / 100)
float SBUSRescaler::sbusToAngle(uint16_t sbus_value,
                                  const AxisCalib& calib,
                                  float max_angle_deg) {

    float pct = rescaleControl(sbus_value, calib);  // -100 ~ +100
    return pct * (max_angle_deg / 100.0f);
}

// ===== SBUS値 → 角速度 変換 =====
// mid を 0deg/s 基準として角速度 [deg/s] に変換する
// 変換式: rescaleControl() [-100,100] × (max_rate_deg_per_sec / 100)
float SBUSRescaler::sbusToRate(uint16_t sbus_value,
                                const AxisCalib& calib,
                                float max_rate_deg_per_sec) {

    float pct = rescaleControl(sbus_value, calib);  // -100 ~ +100
    return pct * (max_rate_deg_per_sec / 100.0f);
}

// ===== サブトリム角度 計算 =====
// calib.center が標準中心(SBUS_MID)からずれている分を角度 [deg] に変換する
// 変換式: (center - SBUS_MID) × (180° / (max - min))
// 例: center=1024(標準) → 0°, center=1100 → (1100-1024)×(180/1320) ≈ +10.4°
float SBUSRescaler::calcSubtrimAngle(const AxisCalib& calib) {

    float deg_per_unit = 180.0f / static_cast<float>(calib.max - calib.min);
    return static_cast<float>(static_cast<int16_t>(calib.center) - static_cast<int16_t>(SBUS_MID))
           * deg_per_unit;
}

// ===== デッドバンド適用 =====
float SBUSRescaler::applyDeadband(float value, float deadband) {

    if (value > -deadband && value < deadband) {

        return 0.0f;
    }

    return value;
}

// ===== 値をクランプ =====
float SBUSRescaler::clamp(float value, float min_val, float max_val) {

    return std::min(std::max(value, min_val), max_val);
}

// ===== 一括リスケーリング: SBUS生データ -> RescaledSBUSData =====
RescaledSBUSData SBUSRescaler::rescale(const std::array<uint16_t, 18>& sbus_data,
                                        const Thresholds& thresholds) {

    RescaledSBUSData result = {};

    // アナログチャネルのリスケーリング（各軸ごとのキャリブレーション値を使用）
    result.throttle = getThrottle(sbus_data, SBUSChannel::THROTTLE, thresholds);
    result.pitch    = getControl(sbus_data, SBUSChannel::PITCH, thresholds.pitch);
    result.roll     = getControl(sbus_data, SBUSChannel::ROLL,  thresholds.roll);
    result.yaw      = getControl(sbus_data, SBUSChannel::YAW,   thresholds.yaw);

    // AUXスイッチチャネル: SwitchPosition (LOW / MID / HIGH) に変換
    result.arm             = getSwitch(sbus_data, SBUSChannel::ARM,             thresholds);
    result.auto_mission    = getSwitch(sbus_data, SBUSChannel::AUTO_MISSION,    thresholds);
    result.safety          = getSwitch(sbus_data, SBUSChannel::SAFETY,          thresholds);
    result.drop            = getSwitch(sbus_data, SBUSChannel::DROP,            thresholds);
    result.preflight_debug = getSwitch(sbus_data, SBUSChannel::PREFLIGHT_DEBUG, thresholds);
    result.flight_debug    = getSwitch(sbus_data, SBUSChannel::FLIGHT_DEBUG,    thresholds);

    return result;
}

} // namespace nokolat
