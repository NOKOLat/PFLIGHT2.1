#ifndef STATE_UTILITY_HPP
#define STATE_UTILITY_HPP

#include <cstdint>

enum class StateID : uint8_t {

    INIT              = 0,
    CALIBRATION       = 1,
    PRE_ARM           = 2,
    ARM               = 3,
    PRE_FLIGHT        = 4,
    FLIGHT            = 5,
    AUTO_FLIGHT       = 6,
    DIS_ARM           = 7,
    MOTOR_SERVO_TEST  = 8,
    ERROR             = 20,
    EMERGENCY_STOP    = 30,   // StateManager フォールバック専用
    INVALID_STATE     = 255
};

// 状態遷移のエラーコード
enum class StateError : uint8_t {

    NONE                        = 0,
    UPDATE_FAILED_RECOVERABLE   = 1,
    UPDATE_FAILED_CRITICAL      = 2,
    CRITICAL_STOPPED            = 3
};

// 遷移フラグ
enum class StateChange : uint8_t {

    NO_STATE_CHANGE = 0,
    STATE_CHANGE    = 1
};

// 状態遷移の成功/失敗（changeState() の戻り値）
enum class StateChangeResult : uint8_t {

    SUCCESS = 0,
    FAILED  = 1,
};

// update() の処理継続/シャットダウン（StateManager::update() の戻り値）
enum class UpdateResult : uint8_t {

    CONTINUE = 0,
    SHUTDOWN = 1,
};

// evaluateNextState() の戻り値（遷移情報）
struct StateResult{

    StateChange change;
    StateID     next_state;
};


// ログ用ヘルパー
inline const char* StateIDToString(StateID state_id) {

    switch (state_id) {

        case StateID::INIT:           return "INIT";
        case StateID::CALIBRATION:    return "CALIBRATION";
        case StateID::PRE_ARM:        return "PRE_ARM";
        case StateID::ARM:            return "ARM";
        case StateID::PRE_FLIGHT:     return "PRE_FLIGHT";
        case StateID::FLIGHT:         return "FLIGHT";
        case StateID::AUTO_FLIGHT:    return "AUTO_FLIGHT";
        case StateID::DIS_ARM:           return "DIS_ARM";
        case StateID::MOTOR_SERVO_TEST:  return "MOTOR_SERVO_TEST";
        case StateID::ERROR:             return "ERROR";
        case StateID::EMERGENCY_STOP: return "EMERGENCY_STOP";
        case StateID::INVALID_STATE:  return "INVALID_STATE";
        default:                      return "UNKNOWN_STATE";
    }
}

#endif // STATE_UTILITY_HPP
