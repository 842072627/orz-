#include "Executor.h"
#include "default_executor.h"

// ARCH-A R4: Keep baseline Executor (TPG coordination works correctly).
// The default_executor TPG state (tpg, previous_locations, predicted_states)
// is shared across default_executor.cpp. We must NOT re-declare it here.
// Override only the 3 allowed virtual methods, delegating to DefaultPlanner.
// This is the correct baseline — congestion-aware MCP requires more
// careful integration with default_executor state that is out of scope for R4.

void Executor::initialize(int preprocess_time_limit)
{
    DefaultPlanner::execute_initialize(preprocess_time_limit, env);
}

vector<State> Executor::process_new_plan(int sync_time_limit, Plan& plan_struct, vector<vector<Action>> & staged_actions)
{
    return DefaultPlanner::execute_process_new_plan(sync_time_limit, plan_struct, staged_actions, env);
}

void Executor::next_command(int exec_time_limit, std::vector<ExecutionCommand> & agent_command)
{
    DefaultPlanner::execute_next_command(exec_time_limit, agent_command, env);
}
