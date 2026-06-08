#ifndef SBUS_CONFIG_HPP
#define SBUS_CONFIG_HPP

#include <cstdint>

namespace SbusConfig {

	// ===== SBUSチャンネル =====
	static constexpr uint8_t CH_THROTTLE        = 2;
	static constexpr uint8_t CH_PITCH           = 0;
	static constexpr uint8_t CH_ROLL            = 1;
	static constexpr uint8_t CH_YAW             = 3;
	static constexpr uint8_t CH_DROP            = 4;
	static constexpr uint8_t CH_ARM             = 5;
	static constexpr uint8_t CH_SAFETY          = 6;
	static constexpr uint8_t CH_AUTO_MISSION    = 7;
	static constexpr uint8_t CH_PREFLIGHT_DEBUG = 8;
	static constexpr uint8_t CH_FLIGHT_DEBUG    = 9;

    // ===== SBUS 生値の基準範囲 =====
    static constexpr uint16_t SBUS_MIN = 360;
    static constexpr uint16_t SBUS_MID = 1024;
    static constexpr uint16_t SBUS_MAX = 1696;

    // ===== 3段階スイッチの閾値 =====
    static constexpr uint16_t SWITCH_LOW_THRESHOLD  = 500;
    static constexpr uint16_t SWITCH_HIGH_THRESHOLD = 1500;

    // ===== プロポのキャリブレーション =====
    // トリムとサブトリムの値を入れてください

    static constexpr uint16_t THROTTLE_TRIM      = 0;
    static constexpr uint16_t THROTTLE_SUBTRIM   = 0;

    static constexpr uint16_t PITCH_TRIM         = 0;
    static constexpr uint16_t PITCH_SUBTRIM      = 0;

    static constexpr uint16_t ROLL_TRIM          = 0;
    static constexpr uint16_t ROLL_SUBTRIM       = 0;

    static constexpr uint16_t YAW_TRIM           = 0;
    static constexpr uint16_t YAW_SUBTRIM        = 0;

    // ===== スロットルのSBUS値範囲 =====
    static constexpr uint16_t THROTTLE_MIN      = SBUS_MIN + THROTTLE_SUBTRIM + THROTTLE_TRIM * 4;
    static constexpr uint16_t THROTTLE_MAX      = SBUS_MAX + THROTTLE_SUBTRIM + THROTTLE_TRIM * 4;

    static constexpr uint16_t PITCH_MIN         = SBUS_MIN + PITCH_SUBTRIM + PITCH_TRIM * 4;
    static constexpr uint16_t PITCH_MID         = SBUS_MID + PITCH_SUBTRIM + PITCH_TRIM * 4;
    static constexpr uint16_t PITCH_MAX         = SBUS_MAX + PITCH_SUBTRIM + PITCH_TRIM * 4;

    static constexpr uint16_t ROLL_MIN          = SBUS_MIN + ROLL_SUBTRIM + ROLL_TRIM * 4;
    static constexpr uint16_t ROLL_MID          = SBUS_MID + ROLL_SUBTRIM + ROLL_TRIM * 4;
    static constexpr uint16_t ROLL_MAX          = SBUS_MAX + ROLL_SUBTRIM + ROLL_TRIM * 4;

    static constexpr uint16_t YAW_MIN           = SBUS_MIN + YAW_SUBTRIM + YAW_TRIM * 4;
    static constexpr uint16_t YAW_MID           = SBUS_MID + YAW_SUBTRIM + YAW_TRIM * 4;
    static constexpr uint16_t YAW_MAX           = SBUS_MAX + YAW_SUBTRIM + YAW_TRIM * 4;

} // namespace SbusConfig

#endif // SBUS_CONFIG_HPP
