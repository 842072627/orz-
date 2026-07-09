// optsvr/executor/ConservativeIncrementalTpg/ConservativeIncrementalTpgExecutor.h
// Conservative mode=8 wrapper for the 9f9bf33 IncrementalTpgExecutor behavior.
#pragma once

#include "Executor.h"

namespace orz_engine {

class ConservativeIncrementalTpgExecutor {
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

    bool is_tpg_front(const std::vector<std::list<int>>& queues,
                      int location,
                      int agent_id) const;
};

} // namespace orz_engine
