// optsvr/executor/Origin/OriginExecutor.h
// Origin 模式：自适应 append + MCP 跟随链
// EXECUTOR_MODE=1 时使用
#pragma once

#include "Executor.h"
#include "RuntimeConfig.h"
#include <vector>

namespace optsvr {

class OriginExecutor {
public:
    // 在 Executor::initialize 中调用
    void initialize(Executor* exec);
    bool is_initialized() const { return exec_ != nullptr; }

    // process_new_plan: 自适应 append 步数
    std::vector<State> process_new_plan(
        int sync_time_limit, Plan& plan_struct,
        std::vector<std::vector<Action>>& staged_actions);

    // next_command: MCP 跟随链决策
    void next_command(int exec_time_limit, std::vector<ExecutionCommand>& agent_command);

private:
    Executor* exec_ = nullptr;
    const SharedEnvironment* env_ = nullptr;

    int turn_ = 0;
    long long total_push_action_count_ = 0;

    // MCP 跟随链
    bool mcp(int agent_id, std::vector<bool>& curr_decision,
             std::vector<ExecutionCommand>& agent_command);

    int following_chain_go_count_ = 0;
};

} // namespace optsvr
