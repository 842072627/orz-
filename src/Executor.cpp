// Executor.cpp — ORZ-only: only uses GameExecutor
#include "Executor.h"
#include "Plan.h"
#include "RuntimeConfig.h"
#include "GameExecutor.h"
#include <algorithm>
#include <cstdio>

static optsvr::GameExecutor g_game_executor;

bool Executor::is_valid_location(int loc) const {
    return loc >= 0 && loc < (int)env->map.size() && env->map[loc];
}

void Executor::initialize(int preprocess_time_limit)
{
    g_game_executor.initialize(this);
}

std::vector<State> Executor::process_new_plan(
    int sync_time_limit, Plan& plan, std::vector<std::vector<Action>>& staged_actions)
{
    return g_game_executor.process_new_plan(sync_time_limit, plan, staged_actions);
}

void Executor::next_command(int exec_time_limit, std::vector<ExecutionCommand>& agent_command)
{
    g_game_executor.next_command(exec_time_limit, agent_command);
}

void Executor::rebuild_from_system_states() {}
bool Executor::mcp_default_executor(int, std::vector<ExecutionCommand>&) { return false; }
