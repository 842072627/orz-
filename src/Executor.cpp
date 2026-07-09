#include "Executor.h"
#include "RuntimeConfig.h"
#include "MapTuner.h"
#include "GameExecutor.h"

static optsvr::GameExecutor g_game_executor;

bool Executor::is_valid_location(int loc) const {
    return loc >= 0 && loc < (int)env->map.size() && env->map[loc];
}

void Executor::initialize(int preprocess_time_limit)
{
    g_game_executor.initialize(this);
}

std::vector<State> Executor::process_new_plan(int steps_to_keep, Plan& plan,
    std::vector<std::vector<Action>>& actions)
{
    return g_game_executor.process_new_plan(steps_to_keep, plan, actions);
}

void Executor::next_command(int exec_time_limit, std::vector<ExecutionCommand>& agent_command)
{
    g_game_executor.next_command(exec_time_limit, agent_command);
}

void Executor::rebuild_from_system_states() {}

bool Executor::mcp_default_executor(int, std::vector<ExecutionCommand>&) { return false; }
