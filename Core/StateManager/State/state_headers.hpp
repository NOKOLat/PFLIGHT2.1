#ifndef STATE_HEADERS_HPP
#define STATE_HEADERS_HPP

#include "../StateInterface/state_interface.hpp"
#include "../common/state_utility.hpp"
#include "../common/state_context.hpp"


// --- Init ---

class InitState : public StateInterface {

    public:
        virtual ~InitState() = default;
        StateError  init(StateContext& context) override;
        StateError  update(StateContext& context) override;
        StateResult evaluateNextState(StateContext& context) override;
        StateID     getStateID() const override;

    private:
        bool initialized_ = false;
};

class CalibrationState : public StateInterface {

    public:
        virtual ~CalibrationState() = default;
        StateError  init(StateContext& context) override;
        StateError  update(StateContext& context) override;
        StateResult evaluateNextState(StateContext& context) override;
        StateID     getStateID() const override;
};


// --- PreArm ---

class PreArmState : public StateInterface {

    public:
        virtual ~PreArmState() = default;
        StateError  init(StateContext& context) override;
        StateError  update(StateContext& context) override;
        StateResult evaluateNextState(StateContext& context) override;
        StateID     getStateID() const override;
};

class ArmState : public StateInterface {

    public:
        virtual ~ArmState() = default;
        StateError  init(StateContext& context) override;
        StateError  update(StateContext& context) override;
        StateResult evaluateNextState(StateContext& context) override;
        StateID     getStateID() const override;
};

class PreFlightState : public StateInterface {

    public:
        virtual ~PreFlightState() = default;
        StateError  init(StateContext& context) override;
        StateError  update(StateContext& context) override;
        StateResult evaluateNextState(StateContext& context) override;
        StateID     getStateID() const override;
};

class MotorServoTestState : public StateInterface {

    public:
        virtual ~MotorServoTestState() = default;
        StateError  init(StateContext& context) override;
        StateError  update(StateContext& context) override;
        StateResult evaluateNextState(StateContext& context) override;
        StateID     getStateID() const override;

    private:
        uint16_t counter_       = 0;
        uint8_t  current_index_ = 0;   // テスト中のサーボ/モーターインデックス
        uint8_t  servo_count_   = 0;   // キャッシュしたサーボ数
        uint8_t  motor_count_   = 0;   // キャッシュしたモーター数

        // 400Hz × 1秒 = 400カウント
        static constexpr uint16_t ONE_SECOND_COUNT = 400;
};


// --- Flight ---

class FlightStateBase : public StateInterface {

    public:
        virtual ~FlightStateBase() = default;
        StateError  init(StateContext& context) final;
        StateError  update(StateContext& context) final;
        StateResult evaluateNextState(StateContext& context) final;

    protected:
        virtual StateError  onInit(StateContext& context)              = 0;
        virtual StateError  onUpdate(StateContext& context)            = 0;
        virtual StateResult onEvaluateNextState(StateContext& context) = 0;

    private:
        bool pwm_tick_ = false;
};

class FlightState : public FlightStateBase {

    public:
        virtual ~FlightState() = default;
        StateID     getStateID() const override;

    protected:
        StateError  onInit(StateContext& context) override;
        StateError  onUpdate(StateContext& context) override;
        StateResult onEvaluateNextState(StateContext& context) override;
};

class AutoFlightState : public FlightStateBase {

    public:
        virtual ~AutoFlightState() = default;
        StateID     getStateID() const override;

    protected:
        StateError  onInit(StateContext& context) override;
        StateError  onUpdate(StateContext& context) override;
        StateResult onEvaluateNextState(StateContext& context) override;
};


// --- PostFlight ---

class DisArmState : public StateInterface {

    public:
        virtual ~DisArmState() = default;
        StateError  init(StateContext& context) override;
        StateError  update(StateContext& context) override;
        StateResult evaluateNextState(StateContext& context) override;
        StateID     getStateID() const override;
};


// --- Error ---

class ErrorState : public StateInterface {

    public:
        virtual ~ErrorState() = default;
        StateError  init(StateContext& context) override;
        StateError  update(StateContext& context) override;
        StateResult evaluateNextState(StateContext& context) override;
        StateID     getStateID() const override;
};

// EMERGENCY_STOP: StateManager フォールバック専用
class EmergencyStopState : public StateInterface {

    public:
        virtual ~EmergencyStopState() = default;
        StateError  init(StateContext& context) override;
        StateError  update(StateContext& context) override;
        StateResult evaluateNextState(StateContext& context) override;
        StateID     getStateID() const override;
};


#endif // STATE_HEADERS_HPP
