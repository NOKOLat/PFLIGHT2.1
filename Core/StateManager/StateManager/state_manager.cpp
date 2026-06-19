#include "state_manager.hpp"
#include "../StateFactory/state_factory.hpp"
#include "SBUS/sbus_manager.hpp"
#include <cstdio>


StateManager::StateManager(StateID init_state_id): init_state_id_(init_state_id) {

    state_context_.publish_log = [](const std::string& msg) {

        printf("%s\n", msg.c_str());
    };
}


bool StateManager::init() {

    fallback_emergency_stop_ = StateFactory::createState(StateID::EMERGENCY_STOP);

    if (!fallback_emergency_stop_) {

        return false;
    }

    nokolat::SbusManager::begin(state_context_.sbus_receiver, state_context_.debug_sbus_uart);

    //nokolat::SbusManager::begin(state_context_.sbus_receiver, state_context_.sbus_uart);
    state_context_.publish_log("[StateManager] SBUS Ready");

    return true;
}


UpdateResult StateManager::update() {

    if (!current_state_ && !use_fallback_) {

        if(changeState(init_state_id_) == StateChangeResult::FAILED) {

            enterFallback("[StateManager] CRITICAL: Failed to create initial state. Falling back to EMERGENCY_STOP.");
        }

        return UpdateResult::CONTINUE;
    }

    updateSbusStatus();

    if(use_fallback_) {

        return handleFallback();
    }

    StateError update_error = current_state_->update(state_context_);

    if(update_error != StateError::NONE) {

        return handleStateError(update_error, current_state_->getStateID());
    }

    StateResult result = current_state_->evaluateNextState(state_context_);

    if(result.change == StateChange::STATE_CHANGE) {

        if(changeState(result.next_state) == StateChangeResult::FAILED) {

            enterFallback("[StateManager] State transition failed. Falling back to EMERGENCY_STOP.");
            return UpdateResult::CONTINUE;
        }
    }

    return UpdateResult::CONTINUE;
}


void StateManager::enterFallback(const std::string& reason) {

    if(!use_fallback_) {

        state_context_.publish_log(reason);
    }

    use_fallback_ = true;
}


bool StateManager::updateSbusStatus() {

    nokolat::SbusUpdateResult sbus_result = nokolat::SbusManager::update(state_context_.sbus_receiver, state_context_.sbus_data);

    if(sbus_result.timeout) {
        enterFallback("[StateManager] SBUS timeout detected. Falling back to EMERGENCY_STOP.");

        return false;
    }

    if(sbus_result.failsafe) {

        enterFallback("[StateManager] SBUS failsafe detected. Falling back to EMERGENCY_STOP.");

        return false;
    }

    return true;
}


UpdateResult StateManager::handleStateError(StateError error, StateID state_id) {

    switch(error) {

        case StateError::NONE:
            return UpdateResult::CONTINUE;

        case StateError::UPDATE_FAILED_RECOVERABLE:
            state_context_.publish_log("[StateManager] Recoverable state update error (ID: " + std::string(StateIDToString(state_id)) + ")");
            return UpdateResult::CONTINUE;

        case StateError::UPDATE_FAILED_CRITICAL:
            enterFallback("[StateManager] CRITICAL: State update failed (ID: " + std::string(StateIDToString(state_id)) + "). Falling back to EMERGENCY_STOP.");
            return UpdateResult::CONTINUE;

        case StateError::CRITICAL_STOPPED:
            state_context_.publish_log("[StateManager] Critical stop completed (ID: " + std::string(StateIDToString(state_id)) + "). Shutting down.");
            while(1);
            return UpdateResult::SHUTDOWN;
    }

    enterFallback("[StateManager] Unknown state error. Falling back to EMERGENCY_STOP.");

    return UpdateResult::CONTINUE;
}


UpdateResult StateManager::handleFallback() {

    if(!fallback_emergency_stop_) {

        state_context_.publish_log("[StateManager] CRITICAL: EMERGENCY_STOP state is not available. Shutting down.");
        while(1);
        return UpdateResult::SHUTDOWN;
    }

    StateError fallback_error = fallback_emergency_stop_->update(state_context_);

    return handleStateError(fallback_error, StateID::EMERGENCY_STOP);
}


StateChangeResult StateManager::changeState(StateID state_id) {

    std::unique_ptr<StateInterface> new_state = StateFactory::createState(state_id);

    if(!new_state) {

        state_context_.publish_log("[StateManager] Failed to create state instance (ID: " + std::to_string(static_cast<int>(state_id)) + ")");
        return StateChangeResult::FAILED;
    }

    StateError init_error = new_state->init(state_context_);

    if(init_error != StateError::NONE) {

        state_context_.publish_log("[StateManager] State init() failed (ID: " + std::string(StateIDToString(state_id)) + ")");
        return StateChangeResult::FAILED;
    }

    current_state_ = std::move(new_state);
    state_context_.publish_log("[StateManager] Next state: " + std::string(StateIDToString(state_id)));

    return StateChangeResult::SUCCESS;
}


StateContext& StateManager::getContext() {

    return state_context_;
}


StateID StateManager::getCurrentStateID() const {

    if(!current_state_) {

        return StateID::INVALID_STATE;
    }

    return current_state_->getStateID();
}
