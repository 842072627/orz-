// Executor.cpp - R306 Reversed Priority
// 1. Call default executor for base GO/STOP decisions
// 2. Apply R306 post-processing: when two agents block each other,
//    let the agent FURTHER from its goal proceed (clears congestion faster)

#include "Executor.h"
#include "default_executor.h"
#include <cmath>

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
    // Step 1: Let the default executor resolve most conflicts
    DefaultPlanner::execute_next_command(exec_time_limit, agent_command, env);

    // Step 2: R306 Reversed Priority post-processing
    // Find all FW agents and detect mutual blocks, then apply reversed priority
    int n = env->num_of_agents;
    int moves[4] = {1, env->cols, -1, -env->cols};

    // Build next-location map: next_loc[loc] = agent_id at that next loc (or -1)
    std::vector<int> next_loc(env->rows * env->cols, -1);
    for (int i = 0; i < n; i++) {
        if (agent_command[i] == ExecutionCommand::GO &&
            !env->staged_actions[i].empty() &&
            env->staged_actions[i].front() == Action::FW) {
            int curr = env->system_states[i].location;
            int ori = env->system_states[i].orientation;
            int nl = curr + moves[ori];
            if (nl >= 0 && nl < (int)next_loc.size())
                next_loc[nl] = i;
        }
    }

    // For each agent that was told to STOP (might be blocked),
    // check if it's a mutual block and apply R306
    std::vector<bool> revised(n, false);
    for (int pass = 0; pass < 5; pass++) {
        bool any = false;
        for (int i = 0; i < n; i++) {
            if (revised[i]) continue;
            if (agent_command[i] == ExecutionCommand::GO) continue;
            if (env->staged_actions[i].empty() || env->staged_actions[i].front() != Action::FW) continue;

            int curr = env->system_states[i].location;
            int ori = env->system_states[i].orientation;
            int nl = curr + moves[ori];
            if (nl < 0 || nl >= (int)next_loc.size()) continue;

            int blocker = next_loc[nl];
            if (blocker == -1 || blocker == i) { revised[i] = true; continue; }
            if (revised[blocker]) { revised[i] = true; continue; }

            // Check if the blocking relationship is mutual
            // (blocker is going somewhere, and its next lands on i's current or beyond)
            bool blocker_fw = (agent_command[blocker] == ExecutionCommand::GO &&
                               !env->staged_actions[blocker].empty() &&
                               env->staged_actions[blocker].front() == Action::FW);
            if (!blocker_fw) { revised[i] = true; continue; }

            // Mutual block: compare distances to goal
            int b_curr = env->system_states[blocker].location;
            int b_ori = env->system_states[blocker].orientation;
            int b_nl = b_curr + moves[b_ori];

            bool mutual = (b_nl >= 0 && b_nl < (int)next_loc.size() && next_loc[b_nl] == blocker);

            if (mutual) {
                int my_dist = 0, blk_dist = 0;
                if (i < (int)env->goal_locations.size() && !env->goal_locations[i].empty())
                    my_dist = std::abs(nl - env->goal_locations[i].front().first);
                if (blocker < (int)env->goal_locations.size() && !env->goal_locations[blocker].empty())
                    blk_dist = std::abs(b_curr - env->goal_locations[blocker].front().first);

                if (my_dist > blk_dist) {
                    // i is further -> i gets GO, blocker gets STOP
                    agent_command[i] = ExecutionCommand::GO;
                    agent_command[blocker] = ExecutionCommand::STOP;
                    revised[i] = true;
                    revised[blocker] = true;
                    // Update next_loc to reflect new decisions
                    if (b_nl >= 0 && b_nl < (int)next_loc.size()) next_loc[b_nl] = -1;
                    any = true;
                } else {
                    revised[i] = true;
                }
            } else {
                revised[i] = true;
            }
        }
        if (!any) break;
    }
}
