// optsvr/executor/RoomExecutor/RoomExecutor.h
// 来源: IDEA/room-0628 dh_executor_occupancy_only_following_preview
// Room 算例专用执行器：occupancy-only preview + following
// EXECUTOR_MODE=5 时使用
#pragma once

#include "Executor.h"
#include <list>
#include <unordered_map>
#include <vector>

namespace optsvr {

class RoomExecutor {
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

    // TPG: location -> occupancy preview visit order
    std::unordered_map<int, std::list<int>> tpg_;
    std::unordered_map<int, std::list<int>> temp_tpg_;

    std::vector<State> predicted_states_;
    int window_size_ = 1;
};

} // namespace optsvr
