// optsvr/executor/RoomOpt/RoomOptExecutor.h
// 来源: IDEA/RoomOpt/Executor.cpp (v10.25)
// 连续 STOP 截断策略：只有 agent 连续 2+ tick 被 STOP 才截断 staged_actions
// 避免瞬时 1-tick 阻塞导致误截断
// EXECUTOR_MODE=4 时使用
#pragma once

#include "Executor.h"
#include <vector>
#include <list>
#include <unordered_map>

namespace optsvr {

class RoomOptExecutor {
public:
    void initialize(Executor* exec);
    bool is_initialized() const { return exec_ != nullptr; }

    std::vector<State> process_new_plan(
        int sync_time_limit, Plan& plan_struct,
        std::vector<std::vector<Action>>& staged_actions);

    void next_command(int exec_time_limit, std::vector<ExecutionCommand>& agent_command);

private:
    Executor* exec_ = nullptr;
    SharedEnvironment* env_ = nullptr;

    // TPG (与 IDEA 原始实现一致: unordered_map<int, list<int>>)
    std::unordered_map<int, std::list<int>> tpg_;
    std::unordered_map<int, std::list<int>> temp_tpg_;

    std::vector<State> predicted_states_;
    std::vector<int> previous_locations_;
    int window_size_ = 1;
    bool first_execution_ = true;

    // 连续 STOP 计数器
    std::vector<int> consecutive_stop_;

    // 截断阈值（连续 STOP 达到此值后截断），可通过 OPT_ROOM_TRUNC_THRESH 覆盖
    int truncate_threshold_ = 2;
};

} // namespace optsvr
