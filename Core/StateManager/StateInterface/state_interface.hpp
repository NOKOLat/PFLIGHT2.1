#ifndef STATE_INTERFACE_HPP
#define STATE_INTERFACE_HPP

#include "../common/state_utility.hpp"
#include "../common/state_context.hpp"

class StateInterface {

    public:

        virtual ~StateInterface() = default;

        // [in] context : 状態間共有データ
        // [return] NONE（成功）or UPDATE_FAILED_CRITICAL（致命的）
        virtual StateError  init(StateContext& context)              = 0;

        // [in] context : 状態間共有データ
        // [return] NONE（成功）or UPDATE_FAILED_CRITICAL（致命的）
        virtual StateError  update(StateContext& context)            = 0;

        // [in] context : 状態間共有データ
        // [return] StateResult（遷移フラグ、次状態）
        virtual StateResult evaluateNextState(StateContext& context) = 0;

        virtual StateID     getStateID() const                       = 0;

    protected:

        uint8_t sub_step_ = 0;
};

#endif // STATE_INTERFACE_HPP
