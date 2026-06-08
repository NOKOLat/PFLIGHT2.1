#ifndef STATE_MANAGER_HPP
#define STATE_MANAGER_HPP

#include <memory>
#include "../StateInterface/state_interface.hpp"
#include "../common/state_utility.hpp"
#include "../common/state_context.hpp"

class StateManager {

    public:

        explicit StateManager(StateID init_state_id);
        virtual ~StateManager() = default;

        // 初期化処理（最初に1回だけ呼ぶ）
        bool init();

        // メインループから毎フレーム呼ぶ
        UpdateResult update();

        // StateContext へのアクセス（CommunicationManager が使用）
        StateContext& getContext();

        // 現在の状態 ID を取得
        StateID getCurrentStateID() const;


    private:

        StateID                         init_state_id_;
        std::unique_ptr<StateInterface> current_state_;
        std::unique_ptr<StateInterface> fallback_emergency_stop_;
        bool                            use_fallback_ = false;
        StateContext                    state_context_;

        StateChangeResult changeState(StateID state_id);
};


#endif // STATE_MANAGER_HPP
