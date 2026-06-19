#include "state_manager.hpp"
#include "../StateFactory/state_factory.hpp"
#include "SBUS/sbus_manager.hpp"
#include <cstdio>

// コンストラクタ
StateManager::StateManager(StateID init_state_id): init_state_id_(init_state_id) {

    state_context_.publish_log = [](const std::string& msg) {

        printf("%s\n", msg.c_str());
    };
}

// 初期化処理
bool StateManager::init() {

    // 初期状態の作成
    current_state_ = StateFactory::createState(init_state_id_);
    if(!current_state_) {

        state_context_.publish_log("[StateManager] Failed to create initial state instance (ID: " + std::to_string(static_cast<int>(init_state_id_)) + ")");
        return false;
    }

    // SBUSの初期化
    nokolat::SbusManager::begin(state_context_.sbus_receiver, state_context_.debug_sbus_uart);
    //nokolat::SbusManager::begin(state_context_.sbus_receiver, state_context_.sbus_uart);
    state_context_.publish_log("[StateManager] SBUS Ready");

    return true;
}

// 更新処理
UpdateResult StateManager::update() {

    // SBUSの更新
    bool sbus_updated = updateSbusStatus();
    if(!sbus_updated) {

        state_context_.publish_log("[StateManager] SBUS update failed. Staying in current state.");
        return UpdateResult::CONTINUE;
    }

    // 状態の更新
    StateError update_error = current_state_->update(state_context_);
    if(update_error != StateError::NONE) {

        state_context_.publish_log("[StateManager] State update() failed. Staying in current state.");
        return UpdateResult::CONTINUE;
    }

    // 状態遷移
    StateResult result = current_state_->evaluateNextState(state_context_);
    if(result.change == StateChange::STATE_CHANGE) {

        if(changeState(result.next_state) == StateChangeResult::FAILED) {

            state_context_.publish_log("[StateManager] State transition failed. Staying in current state.");
            return UpdateResult::CONTINUE;
        }
    }

    return UpdateResult::CONTINUE;
}

// SBUS処理の更新
bool StateManager::updateSbusStatus() {

    // SBUSの更新
    nokolat::SbusUpdateResult sbus_result = nokolat::SbusManager::update(state_context_.sbus_receiver, state_context_.sbus_data);

    // 受信失敗
    if(sbus_result.timeout) {

        state_context_.publish_log("[StateManager] SBUS timeout detected. Staying in current state.");
        return false;
    }

    // 受信切断
    if(sbus_result.failsafe) {

        state_context_.publish_log("[StateManager] SBUS failsafe detected. Staying in current state.");
        return false;
    }

    return true;
}

// 状態変更処理
StateChangeResult StateManager::changeState(StateID state_id) {

    // 新しい状態の作成
    std::unique_ptr<StateInterface> new_state = StateFactory::createState(state_id);
    if(!new_state) {

        state_context_.publish_log("[StateManager] Failed to create state instance (ID: " + std::to_string(static_cast<int>(state_id)) + ")");
        return StateChangeResult::FAILED;
    }

    // 新しい状態の初期化
    StateError init_error = new_state->init(state_context_);
    if(init_error != StateError::NONE) {

        state_context_.publish_log("[StateManager] State init() failed (ID: " + std::string(StateIDToString(state_id)) + ")");
        return StateChangeResult::FAILED;
    }

    // 状態の切り替え
    current_state_ = std::move(new_state);
    state_context_.publish_log("[StateManager] Next state: " + std::string(StateIDToString(state_id)));

    return StateChangeResult::SUCCESS;
}
