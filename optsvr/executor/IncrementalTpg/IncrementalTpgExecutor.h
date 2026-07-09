// optsvr/executor/IncrementalTpg/IncrementalTpgExecutor.h
// Incremental TPG executor: builds executable reservations while simulating.
// EXECUTOR_MODE=7.
#pragma once

#include "Executor.h"

namespace optsvr {

class IncrementalTpgExecutor {
public:
    void initialize(Executor* exec);
    bool is_initialized() const { return exec_ != nullptr; }

    std::vector<State> process_new_plan(
        int sync_time_limit,
        Plan& plan_struct,
        std::vector<std::vector<Action>>& staged_actions);

    void next_command(
        int exec_time_limit,
        std::vector<ExecutionCommand>& agent_command);

private:
    Executor* exec_ = nullptr;
    SharedEnvironment* env_ = nullptr;
};

} // namespace optsvr
