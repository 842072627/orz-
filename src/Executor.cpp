#include "Executor.h"
#include "IronPlatoonExecutor.h"
#include "MapTuner.h"
#include "RuntimeConfig.h"

#include <algorithm>

namespace {
optsvr::IronPlatoonExecutor game_executor;
}

void Executor::initialize(int) {
    optsvr::load_map_tuning(env);
    if (optsvr::cfg().skip_test) return;
    game_executor.initialize(this);
}

vector<State> Executor::process_new_plan(
    int sync_time_limit, Plan& plan, vector<vector<Action>>& staged_actions) {
    if (optsvr::cfg().skip_test) {
        staged_actions.assign(env->num_of_agents, {});
        return env->system_states;
    }
    if (!game_executor.is_initialized()) game_executor.initialize(this);
    return game_executor.process_new_plan(sync_time_limit, plan, staged_actions);
}

void Executor::next_command(
    int exec_time_limit, std::vector<ExecutionCommand>& agent_command) {
    if (optsvr::cfg().skip_test) {
        agent_command.assign(env->num_of_agents, ExecutionCommand::STOP);
        return;
    }
    if (!game_executor.is_initialized()) game_executor.initialize(this);
    game_executor.next_command(exec_time_limit, agent_command);
}
