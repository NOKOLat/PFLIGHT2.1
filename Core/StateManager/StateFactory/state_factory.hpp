#ifndef STATE_FACTORY_HPP
#define STATE_FACTORY_HPP

#include <memory>
#include "../StateInterface/state_interface.hpp"
#include "../common/state_utility.hpp"
#include "../State/state_headers.hpp"


class StateFactory {

    public:

        static std::unique_ptr<StateInterface> createState(StateID state_id) {

            switch (state_id) {

                // --- Init ---
                case StateID::INIT:
                    return std::make_unique<InitState>();

                case StateID::CALIBRATION:
                    return std::make_unique<CalibrationState>();

                // --- PreArm ---
                case StateID::PRE_ARM:
                    return std::make_unique<PreArmState>();

                case StateID::ARM:
                    return std::make_unique<ArmState>();

                case StateID::PRE_FLIGHT:
                    return std::make_unique<PreFlightState>();

                case StateID::MOTOR_SERVO_TEST:
                    return std::make_unique<MotorServoTestState>();

                // --- Flight ---
                case StateID::FLIGHT:
                    return std::make_unique<FlightState>();

                case StateID::AUTO_FLIGHT:
                    return std::make_unique<AutoFlightState>();

                // --- PostFlight ---
                case StateID::DIS_ARM:
                    return std::make_unique<DisArmState>();

                // --- Error ---
                case StateID::ERROR:
                    return std::make_unique<ErrorState>();

                // --- フォールバック専用 ---
                case StateID::EMERGENCY_STOP:
                    return std::make_unique<EmergencyStopState>();

                default:
                    return std::make_unique<EmergencyStopState>();
            }
        }
};


#endif // STATE_FACTORY_HPP
