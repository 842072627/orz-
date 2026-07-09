// optsvr/executor/GameExecutor/GameExecutor.h
// 来源: 基于 DHExecutor(mode=3) 全量继承 + RoomOptExecutor(mode=4) 截断策略
// EXECUTOR_MODE=6 时使用
#pragma once

#include "Executor.h"
#include "PibtLock.h"
#include <list>
#include <unordered_map>
#include <vector>

namespace nz {

class GameExecutor {
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

    // ---- TPG (与 DHExecutor 完全一致) ----
    std::unordered_map<int, std::list<int>> tpg_;
    std::unordered_map<int, std::list<int>> temp_tpg_;

    std::vector<State> predicted_states_;
    std::vector<int> previous_locations_;
    int window_size_ = 1;
    bool first_execution_ = true;

    // ---- promote_temporary_pass 常量 (与 DHExecutor 一致) ----
    static constexpr int TEMPORARY_PASS_MIN_ACTIONS_AWAY = 3;
    static constexpr int TEMPORARY_PASS_NOT_REACHED = 1000000000;

    // ---- 连续 STOP 截断 (来自 RoomOptExecutor) ----
    std::vector<int> consecutive_stop_;
    int truncate_threshold_ = 2;

    bool enable_truncate_ = true;             // 连续STOP截断开关

    // ---- 辅助函数 (与 DHExecutor 完全一致) ----
    static int get_next_location(const State& state, SharedEnvironment* env);

    static int planned_actions_to_location(
        int agent_id, int target_location,
        const std::vector<State>& states,
        const std::vector<std::vector<Action>>& staged_actions,
        const std::vector<int>& action_index,
        SharedEnvironment* env);

    static int planned_actions_to_leave_location_after_entry(
        int agent_id, int entered_location,
        const std::vector<State>& states,
        const std::vector<std::vector<Action>>& staged_actions,
        const std::vector<int>& action_index,
        SharedEnvironment* env);

    static bool location_is_currently_empty(
        int location, int agent_id, const std::vector<State>& states);

    bool promote_temporary_pass_if_safe(
        int location, int agent_id,
        const std::vector<State>& states,
        const std::vector<std::vector<Action>>& staged_actions,
        const std::vector<int>& action_index);

    // MCP 决策 (含连续STOP截断)
    void mcp(int agent_id, std::vector<ExecutionCommand>& agent_command);
};

} // namespace nz
