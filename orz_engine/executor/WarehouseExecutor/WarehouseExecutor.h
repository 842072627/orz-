// optsvr/executor/WarehouseExecutor/WarehouseExecutor.h
// 来源: optsvr/executor/DHExecutor
// Warehouse 执行器：TPG + windowed append + promote_temporary_pass_if_safe
// EXECUTOR_MODE=9 时使用
#pragma once

#include "Executor.h"
#include <list>
#include <unordered_map>
#include <vector>

namespace orz_engine {

class WarehouseExecutor {
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

    // TPG: location → 访问顺序队列 (使用 unordered_map，与 DHFULL 原始实现一致)
    std::unordered_map<int, std::list<int>> tpg_;
    std::unordered_map<int, std::list<int>> temp_tpg_;

    std::vector<State> predicted_states_;
    std::vector<int> previous_locations_;
    int window_size_ = 1;
    bool first_execution_ = true;

    // ---- promote_temporary_pass 相关常量 ----
    static constexpr int TEMPORARY_PASS_MIN_ACTIONS_AWAY = 3;
    static constexpr int TEMPORARY_PASS_NOT_REACHED = 1000000000;

    // ---- 辅助函数 ----
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

    // MCP 决策
    void mcp(int agent_id, std::vector<ExecutionCommand>& agent_command);
};

} // namespace orz_engine
